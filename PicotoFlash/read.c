#include "read.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

// Read sizes
const size_t k_read_sizes[NUM_READ_SIZES] = {1, 256, 4096, 32768, 65536};
const char* k_read_labels[NUM_READ_SIZES] = {"1-byte", "page", "sector", "block32k", "block64k"};

// ITERATION COUNT - Change this to 100 when you want more iterations
#define ITERS_READ 10

// Global results storage
read_result_t g_read_results[8];
int g_read_result_count = 0;

// NEW: Global variable to store derived 50MHz read speed
static double g_derived_50mhz_speed = 0.0;

// SPI helper functions
static uint32_t g_spi_hz = 0;

static uint32_t spi_set_hz(spi_inst_t *spi, uint32_t hz) {
    uint32_t actual = spi_set_baudrate(spi, hz);
    g_spi_hz = actual;
    printf("  [SPI] req=%u MHz, actual=%u MHz\n",
           (unsigned)(hz / 1000000u), (unsigned)(actual / 1000000u));
    return actual;
}

static inline void cs_low(uint8_t pin) { gpio_put(pin, 0); }
static inline void cs_high(uint8_t pin) { gpio_put(pin, 1); }
static inline void spi_tx(spi_inst_t *spi, const uint8_t *b, size_t n) { spi_write_blocking(spi, b, n); }
static inline void spi_rx(spi_inst_t *spi, uint8_t *b, size_t n) { spi_read_blocking(spi, 0x00, b, n); }

// Flash read functions
static void flash_read03(spi_inst_t *spi, uint8_t cs_pin, uint32_t addr, uint8_t *buf, size_t len) {
    uint8_t h[4] = {0x03, (uint8_t)(addr >> 16), (uint8_t)(addr >> 8), (uint8_t)addr};
    cs_low(cs_pin);
    spi_tx(spi, h, 4);
    spi_rx(spi, buf, len);
    cs_high(cs_pin);
}

static void flash_read0B(spi_inst_t *spi, uint8_t cs_pin, uint32_t addr, uint8_t *buf,
                          size_t len, uint8_t dummy) {
    uint8_t h[5] = {0x0B, (uint8_t)(addr >> 16), (uint8_t)(addr >> 8), (uint8_t)addr, 0x00};
    cs_low(cs_pin);
    spi_tx(spi, h, 5);
    if (dummy > 1) {
        uint8_t d[8] = {0};
        spi_rx(spi, d, dummy - 1);
    }
    spi_rx(spi, buf, len);
    cs_high(cs_pin);
}

// Print helpers
static void print_divider(int width) {
    for (int i = 0; i < width; i++) putchar('-');
    putchar('\n');
}

static void print_section(const char *title) {
    putchar('\n');
    print_divider(72);
    printf("%s\n", title);
    print_divider(72);
}

static void print_table_header(int mhz) {
    char title[64];
    snprintf(title, sizeof title, "READ BENCHMARK @ %d MHz", mhz);
    print_section(title);
    printf("size       | n   | avg(us)    | MB/s\n");
    print_divider(50);
}

static void print_table_row(const char *label, int n, const read_stats_t *s) {
    printf("%-10s | %3d | %10.3f | %7.6f\n",
           label, n, s->avg_us, s->mb_s);
    print_divider(50);
}

// Results management
void read_reset_results(void) {
    g_read_result_count = 0;
    memset(g_read_results, 0, sizeof(g_read_results));
    g_derived_50mhz_speed = 0.0;  // Reset derived speed
}

void read_save_result(int mhz, const read_bench_capture_t *cap) {
    if (g_read_result_count >= 8) return;
    read_result_t *r = &g_read_results[g_read_result_count++];
    r->clock_mhz = mhz;
    r->valid = cap->filled;
    if (cap->filled) {
        for (size_t i = 0; i < NUM_READ_SIZES; i++) {
            r->size_stats[i] = cap->rows[i].stats;
        }
    }
}

// Main benchmark function - TIME ENTIRE BATCH
void read_run_benches_capture(spi_inst_t *spi, uint8_t cs_pin, bool use_fast,
                               uint8_t dummy, int mhz_req, read_bench_capture_t *cap_out) {
    uint8_t *buf = (uint8_t *)malloc(65536);
    if (!buf) {
        printf("[ERR] NOMEM\n");
        return;
    }

    uint32_t actual = spi_set_hz(spi, (uint32_t)mhz_req * 1000u * 1000u);
    int mhz_to_print = (int)(actual / 1000000u);
    
    print_table_header(mhz_to_print);
    cap_out->actual_mhz = mhz_to_print;
    
    for (size_t si = 0; si < NUM_READ_SIZES; ++si) {
        size_t sz = k_read_sizes[si];
        
        // Time the ENTIRE batch of operations
        uint64_t t0 = time_us_64();
        for (int i = 0; i < ITERS_READ; i++) {
            if (use_fast)
                flash_read0B(spi, cs_pin, 0, buf, sz, dummy);
            else
                flash_read03(spi, cs_pin, 0, buf, sz);
        }
        uint64_t t1 = time_us_64();
        
        // Calculate average time per operation
        uint32_t total_us = (uint32_t)(t1 - t0);
        double avg_us = (double)total_us / (double)ITERS_READ;
        
        // Create stats structure
        read_stats_t s;
        s.avg_us = avg_us;
        s.std_us = 0.0; // No stddev with batch timing
        s.vmin = total_us;
        s.vmax = total_us;
        s.p25 = avg_us;
        s.p50 = avg_us;
        s.p75 = avg_us;
        
        // Calculate MB/s
        double sec = avg_us / 1e6;
        s.mb_s = (sz / sec) / 1e6;
        
        print_table_row(k_read_labels[si], ITERS_READ, &s);
        
        cap_out->rows[si].size_bytes = sz;
        cap_out->rows[si].stats = s;
    }
    
    cap_out->filled = true;
    
    // Save result for summary
    read_save_result(mhz_to_print, cap_out);
    
    free(buf);
}

// Interpolation helpers
static int find_best_below(const int *mhz, int n) {
    int idx = -1, best = -100000;
    for (int i = 0; i < n; i++) {
        if (mhz[i] < 50 && mhz[i] > best) {
            best = mhz[i];
            idx = i;
        }
    }
    return idx;
}

static int find_best_above(const int *mhz, int n) {
    int idx = -1, best = 1000000;
    for (int i = 0; i < n; i++) {
        if (mhz[i] > 50 && mhz[i] < best) {
            best = mhz[i];
            idx = i;
        }
    }
    return idx;
}

static int find_closest(const int *mhz, int n) {
    int idx = -1;
    int bestd = 1000000;
    for (int i = 0; i < n; i++) {
        int d = abs(mhz[i] - 50);
        if (d < bestd) {
            bestd = d;
            idx = i;
        }
    }
    return idx;
}

static void print_table_header_derived50(void) {
    printf("\n=== DERIVED 50 MHz TABLE ===\n");
    printf("size       | n   | DER.avg(us) | DER.MB/s\n");
    printf("----------+-----+--------------+----------\n");
}

static void print_row_derived50(const char *label, int n, const read_stats_t *src_scaled, double mbps50) {
    printf("%-8s | n=%-3d | %11.3f | %8.6f\n",
           label, n,
           src_scaled->avg_us, mbps50);
}

void read_derive_and_print_50(const int *req_mhz, const read_bench_capture_t *caps, int n_caps) {
    (void)req_mhz;
    int actuals[8];
    const read_bench_capture_t *used[8];
    int m = 0;
    
    for (int i = 0; i < n_caps; i++) {
        if (caps[i].filled) {
            actuals[m] = caps[i].actual_mhz;
            used[m] = &caps[i];
            m++;
        }
    }
    
    if (m == 0) {
        printf("\n#DERIVED_50MHZ_SKIPPED,no_measurements\n");
        g_derived_50mhz_speed = 0.0;
        return;
    }
    
    print_table_header_derived50();
    
    int idx_lo = find_best_below(actuals, m);
    int idx_hi = find_best_above(actuals, m);
    int idx_closest = find_closest(actuals, m);
    
    // Use 4KB (sector) size for 50MHz speed derivation
    size_t sector_index = 2; // index for 4096 bytes
    double mb50_sector = 0.0;
    
    for (size_t si = 0; si < NUM_READ_SIZES; ++si) {
        double mb50 = 0.0;
        
        if (idx_lo >= 0 && idx_hi >= 0) {
            double f_lo = (double)actuals[idx_lo];
            double f_hi = (double)actuals[idx_hi];
            double mb_lo = used[idx_lo]->rows[si].stats.mb_s;
            double mb_hi = used[idx_hi]->rows[si].stats.mb_s;
            
            if (fabs(f_hi - f_lo) < 1e-9) {
                mb50 = (mb_lo + mb_hi) * 0.5;
            } else {
                double t = (50.0 - f_lo) / (f_hi - f_lo);
                mb50 = mb_lo + t * (mb_hi - mb_lo);
            }
        } else {
            int ic = idx_closest;
            double f_c = (double)actuals[ic];
            double mb_c = used[ic]->rows[si].stats.mb_s;
            mb50 = mb_c * (50.0 / f_c);
        }
        
        // Save 4KB sector speed for extraction
        if (si == sector_index) {
            mb50_sector = mb50;
        }
        
        int ib = idx_closest;
        double mb_base = used[ib]->rows[si].stats.mb_s;
        const read_stats_t *sb = &used[ib]->rows[si].stats;
        double scale_time = (mb_base <= 0 || mb50 <= 0) ? 1.0 : (mb_base / mb50);
        
        read_stats_t der = *sb;
        der.avg_us *= scale_time;
        der.mb_s = mb50;
        
        print_row_derived50(k_read_labels[si], ITERS_READ, &der, mb50);
    }
    
    // Store the derived 50MHz speed (4KB sector) for extraction
    g_derived_50mhz_speed = mb50_sector;
    
    printf("\n[INFO] Derived 50MHz read speed (4KB): %.2f MB/s\n", g_derived_50mhz_speed);
}

// NEW: Function to get the derived 50MHz read speed
double read_get_50mhz_speed(void) {
    return g_derived_50mhz_speed;
}

void read_print_summary_tables(void) {
    print_section("READ BENCHMARK SUMMARY - ALL RESULTS");
    
    // READ PERFORMANCE SUMMARY TABLE
    printf("\n=== READ PERFORMANCE SUMMARY (MB/s) ===\n");
    printf("Clock    | 1-byte  | page    | sector  | block32k | block64k\n");
    printf("--------+---------+---------+---------+----------+---------\n");
    
    for (int i = 0; i < g_read_result_count; i++) {
        if (!g_read_results[i].valid) continue;
        printf("%3d MHz | ", g_read_results[i].clock_mhz);
        for (size_t s = 0; s < NUM_READ_SIZES; s++) {
            printf("%7.4f | ", g_read_results[i].size_stats[s].mb_s);
        }
        printf("\n");
    }
    
    printf("--------+---------+---------+---------+----------+---------\n");
    
    // READ TIMING SUMMARY TABLE
    printf("\n=== READ TIMING SUMMARY (avg microseconds) ===\n");
    printf("Clock    | 1-byte  | page    | sector  | block32k | block64k\n");
    printf("--------+---------+---------+---------+----------+---------\n");
    
    for (int i = 0; i < g_read_result_count; i++) {
        if (!g_read_results[i].valid) continue;
        printf("%3d MHz | ", g_read_results[i].clock_mhz);
        for (size_t s = 0; s < NUM_READ_SIZES; s++) {
            printf("%7.1f | ", g_read_results[i].size_stats[s].avg_us);
        }
        printf("\n");
    }
    
    printf("--------+---------+---------+---------+----------+---------\n");
}
