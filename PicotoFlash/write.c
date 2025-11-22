// flash_write_bench.c
// Write/Program benchmark implementation for SPI flash
// BATCH TIMING VERSION - Times entire batch then averages

#include "write.h"
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define PAGE_SIZE 256
#define SECTOR_4K 4096

// ============================================================================
// GLOBAL STORAGE FOR SD LOGGING
// ============================================================================
int g_write_result_count = 0;
write_bench_capture_t g_write_results[8];  // Support up to 8 clock speeds

// ============================================================================
// Internal flash command functions
// ============================================================================
static inline void cs_low(uint8_t pin) { gpio_put(pin, 0); }
static inline void cs_high(uint8_t pin) { gpio_put(pin, 1); }

static void flash_wren(spi_inst_t *spi, uint8_t cs) {
    uint8_t cmd = 0x06;
    cs_low(cs);
    spi_write_blocking(spi, &cmd, 1);
    cs_high(cs);
}

static uint8_t flash_rdsr(spi_inst_t *spi, uint8_t cs) {
    uint8_t cmd = 0x05, status = 0;
    cs_low(cs);
    spi_write_blocking(spi, &cmd, 1);
    spi_read_blocking(spi, 0x00, &status, 1);
    cs_high(cs);
    return status;
}

static bool flash_wait_busy(spi_inst_t *spi, uint8_t cs, uint32_t timeout_ms) {
    uint64_t start = time_us_64();
    while ((flash_rdsr(spi, cs) & 0x01) != 0) {
        if ((time_us_64() - start) > (uint64_t)timeout_ms * 1000) {
            return false;
        }
        sleep_us(200);
    }
    return true;
}

static void flash_erase_sector(spi_inst_t *spi, uint8_t cs, uint32_t addr) {
    flash_wren(spi, cs);
    uint8_t cmd[4] = {0x20, (uint8_t)(addr >> 16), (uint8_t)(addr >> 8), (uint8_t)addr};
    cs_low(cs);
    spi_write_blocking(spi, cmd, 4);
    cs_high(cs);
}

static void flash_page_program(spi_inst_t *spi, uint8_t cs, uint32_t addr,
                               const uint8_t *data, size_t len) {
    if (len > PAGE_SIZE) len = PAGE_SIZE;
    flash_wren(spi, cs);
    uint8_t cmd[4] = {0x02, (uint8_t)(addr >> 16), (uint8_t)(addr >> 8), (uint8_t)addr};
    cs_low(cs);
    spi_write_blocking(spi, cmd, 4);
    spi_write_blocking(spi, data, len);
    cs_high(cs);
}

static void flash_read(spi_inst_t *spi, uint8_t cs, uint32_t addr,
                      uint8_t *buf, size_t len) {
    uint8_t cmd[4] = {0x03, (uint8_t)(addr >> 16), (uint8_t)(addr >> 8), (uint8_t)addr};
    cs_low(cs);
    spi_write_blocking(spi, cmd, 4);
    spi_read_blocking(spi, 0x00, buf, len);
    cs_high(cs);
}

// Print divider line
static void print_divider(int width) {
    for (int i = 0; i < width; i++) putchar('-');
    putchar('\n');
}

// ============================================================================
// Main write benchmark function - BATCH TIMING
// ============================================================================
bool write_bench_run(void *spi_inst, uint8_t cs_pin, int mhz_req,
                    uint32_t base_addr, const size_t *sizes,
                    const char **labels, int num_sizes,
                    int iterations, write_bench_capture_t *capture) {
    spi_inst_t *spi = (spi_inst_t *)spi_inst;
    
    // Set SPI clock
    uint32_t actual_hz = spi_set_baudrate(spi, (uint32_t)mhz_req * 1000000);
    int actual_mhz = (int)(actual_hz / 1000000);
    
    capture->clock_mhz_requested = mhz_req;
    capture->clock_mhz_actual = actual_mhz;
    capture->valid = true;
    capture->num_results = 0;
    
    printf("  [SPI] Write bench: req=%d MHz, actual=%d MHz\n", mhz_req, actual_mhz);
    
    // Allocate test buffer
    uint8_t *test_buf = malloc(65536);
    if (!test_buf) {
        printf("  [ERR] Failed to allocate test buffer\n");
        capture->valid = false;
        return false;
    }
    
    // Fill with test pattern
    for (size_t i = 0; i < 65536; i++) {
        test_buf[i] = (uint8_t)(i ^ (i >> 8));
    }
    
    // Test each size
    for (int si = 0; si < num_sizes; si++) {
        size_t sz = sizes[si];
        if (capture->num_results >= WRITE_TEST_SIZES) break;
        
        write_bench_result_t *result = &capture->results[capture->num_results];
        result->size_bytes = sz;
        result->label = labels[si];
        result->verify_ok = true;
        
        // Calculate sectors needed
        uint32_t bytes_needed = sz * iterations;
        uint32_t sectors_needed = (bytes_needed + SECTOR_4K - 1) / SECTOR_4K;
        printf("  [PREP] Erasing %u sectors for %s write test...\n",
               (unsigned)sectors_needed, labels[si]);
        
        // Erase sectors
        for (uint32_t s = 0; s < sectors_needed; s++) {
            flash_erase_sector(spi, cs_pin, base_addr + (s * SECTOR_4K));
            if (!flash_wait_busy(spi, cs_pin, 5000)) {
                printf("  [WARN] Erase timeout at sector %u\n", (unsigned)s);
            }
        }
        
        // TIME THE ENTIRE BATCH OF WRITE OPERATIONS
        uint32_t addr = base_addr;
        uint64_t batch_start = time_us_64();
        
        for (int iter = 0; iter < iterations; iter++) {
            // Write data in page-sized chunks
            size_t remaining = sz;
            uint32_t current_addr = addr;
            size_t offset = 0;
            
            while (remaining > 0) {
                size_t chunk = (remaining < PAGE_SIZE) ? remaining : PAGE_SIZE;
                flash_page_program(spi, cs_pin, current_addr, test_buf + offset, chunk);
                if (!flash_wait_busy(spi, cs_pin, 100)) {
                    printf("  [WARN] Write timeout at 0x%06X\n", (unsigned)current_addr);
                }
                current_addr += chunk;
                offset += chunk;
                remaining -= chunk;
            }
            addr += sz;
        }
        
        uint64_t batch_end = time_us_64();
        uint32_t total_us = (uint32_t)(batch_end - batch_start);
        
        // Calculate average time per write
        double avg_us = (double)total_us / (double)iterations;
        
        // Verify last write
        uint8_t verify_buf[256];
        addr = base_addr + (sz * (iterations - 1));
        size_t verify_len = (sz < 256) ? sz : 256;
        flash_read(spi, cs_pin, addr, verify_buf, verify_len);
        
        for (size_t v = 0; v < verify_len; v++) {
            if (verify_buf[v] != test_buf[v]) {
                result->verify_ok = false;
                printf("  [WARN] Verify failed at byte %zu for %s\n", v, labels[si]);
                break;
            }
        }
        
        // Create statistics structure (simplified for batch timing)
        result->stats.avg_us = avg_us;
        result->stats.std_us = 0.0;  // No stddev with batch timing
        result->stats.vmin = total_us;
        result->stats.vmax = total_us;
        result->stats.p25 = avg_us;
        result->stats.p50 = avg_us;
        result->stats.p75 = avg_us;
        
        // Calculate MB/s
        double sec = avg_us / 1e6;
        result->stats.mb_s = (sz / sec) / 1e6;
        
        capture->num_results++;
    }
    
    free(test_buf);
    return true;
}

// ============================================================================
// Run benchmarks at multiple clock speeds
// ============================================================================
int write_bench_run_multi_clock(void *spi_inst, uint8_t cs_pin,
                                const int *clocks, int num_clocks,
                                uint32_t base_addr,
                                write_bench_capture_t *captures) {
    // Default test sizes
    const size_t default_sizes[] = {1, 256, 4096, 32768, 65536};
    const char *default_labels[] = {"1-byte", "page", "sector", "block32k", "block64k"};
    const int num_sizes = 5;
    
    int success_count = 0;
    for (int i = 0; i < num_clocks; i++) {
        printf("\n=== WRITE BENCHMARK @ %d MHz (requested) ===\n", clocks[i]);
        
        if (write_bench_run(spi_inst, cs_pin, clocks[i],
                           base_addr + (i * 0x20000),  // Offset each test
                           default_sizes, default_labels, num_sizes,
                           WRITE_ITERS_DEFAULT, &captures[i])) {
            write_bench_print_results(&captures[i]);
            success_count++;
        } else {
            printf("  [ERR] Write benchmark failed at %d MHz\n", clocks[i]);
        }
    }
    
    return success_count;
}

// ============================================================================
// Print results table
// ============================================================================
void write_bench_print_results(const write_bench_capture_t *capture) {
    if (!capture->valid) {
        printf("  [ERR] Invalid capture data\n");
        return;
    }
    
    printf("\nWRITE BENCHMARK RESULTS @ %d MHz\n", capture->clock_mhz_actual);
    printf("size       |  n  |   avg(us)  |  MB/s   | Verify\n");
    print_divider(60);
    
    for (int i = 0; i < capture->num_results; i++) {
        const write_bench_result_t *r = &capture->results[i];
        printf("%-10s | %3d | %10.3f | %7.6f | %s\n",
               r->label, WRITE_ITERS_DEFAULT,
               r->stats.avg_us, r->stats.mb_s,
               r->verify_ok ? "OK" : "FAIL");
    }
    print_divider(60);
}

// ============================================================================
// Print summary comparison table
// ============================================================================
void write_bench_print_summary(const write_bench_capture_t *captures, int num_captures) {
    printf("\n=== WRITE PERFORMANCE SUMMARY ===\n");
    write_bench_print_performance_summary(captures, num_captures);
    printf("\n");
    write_bench_print_timing_summary(captures, num_captures);
}

// Print performance (MB/s) summary
void write_bench_print_performance_summary(const write_bench_capture_t *captures, int num_captures) {
    printf("\nWRITE PERFORMANCE SUMMARY (MB/s)\n");
    printf("Clock    | 1-byte  | page    | sector  | block32k | block64k\n");
    printf("---------+---------+---------+---------+----------+---------\n");
    
    for (int i = 0; i < num_captures; i++) {
        if (!captures[i].valid) continue;
        printf("%3d MHz | ", captures[i].clock_mhz_actual);
        for (int s = 0; s < captures[i].num_results && s < 5; s++) {
            printf("%7.4f | ", captures[i].results[s].stats.mb_s);
        }
        printf("\n");
    }
    printf("---------+---------+---------+---------+----------+---------\n");
}

// Print timing (microseconds) summary
void write_bench_print_timing_summary(const write_bench_capture_t *captures, int num_captures) {
    printf("\nWRITE TIMING SUMMARY (avg microseconds)\n");
    printf("Clock    | 1-byte  | page    | sector  | block32k | block64k\n");
    printf("---------+---------+---------+---------+----------+---------\n");
    
    for (int i = 0; i < num_captures; i++) {
        if (!captures[i].valid) continue;
        printf("%3d MHz | ", captures[i].clock_mhz_actual);
        for (int s = 0; s < captures[i].num_results && s < 5; s++) {
            printf("%7.1f | ", captures[i].results[s].stats.avg_us);
        }
        printf("\n");
    }
    printf("---------+---------+---------+---------+----------+---------\n");
}
