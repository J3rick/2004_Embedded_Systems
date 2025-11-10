#include "erase.h"
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include <math.h>

// ITERATION COUNT - Change this to 1000 when you want more iterations
#define ITERS_ERASE 10

// Global results storage
erase_result_t g_erase_result;

// SPI helper functions
static inline void cs_low(uint8_t pin) { gpio_put(pin, 0); }
static inline void cs_high(uint8_t pin) { gpio_put(pin, 1); }
static inline void spi_tx(spi_inst_t *spi, const uint8_t *b, size_t n) { spi_write_blocking(spi, b, n); }
static inline void spi_rx(spi_inst_t *spi, uint8_t *b, size_t n) { spi_read_blocking(spi, 0x00, b, n); }

// Flash basic operations
static void flash_wren(spi_inst_t *spi, uint8_t cs_pin) {
    uint8_t c = 0x06;
    cs_low(cs_pin);
    spi_tx(spi, &c, 1);
    cs_high(cs_pin);
}

static uint8_t flash_rdsr(spi_inst_t *spi, uint8_t cs_pin) {
    uint8_t c = 0x05, v = 0;
    cs_low(cs_pin);
    spi_tx(spi, &c, 1);
    spi_rx(spi, &v, 1);
    cs_high(cs_pin);
    return v;
}

static uint8_t flash_rdsr2(spi_inst_t *spi, uint8_t cs_pin) {
    uint8_t c = 0x35, v = 0;
    cs_low(cs_pin);
    spi_tx(spi, &c, 1);
    spi_rx(spi, &v, 1);
    cs_high(cs_pin);
    return v;
}

static void flash_wren_sr_volatile(spi_inst_t *spi, uint8_t cs_pin) {
    uint8_t c = 0x50;
    cs_low(cs_pin);
    spi_tx(spi, &c, 1);
    cs_high(cs_pin);
    sleep_us(5);
}

static void flash_wrsr1(spi_inst_t *spi, uint8_t cs_pin, uint8_t sr1) {
    flash_wren(spi, cs_pin);
    uint8_t c = 0x01;
    cs_low(cs_pin);
    spi_tx(spi, &c, 1);
    cs_high(cs_pin);
    (void)sr1;
}

static void flash_wrsr2(spi_inst_t *spi, uint8_t cs_pin, uint8_t sr2) {
    flash_wren(spi, cs_pin);
    uint8_t c = 0x31;
    cs_low(cs_pin);
    spi_tx(spi, &c, 1);
    cs_high(cs_pin);
    (void)sr2;
}

static bool flash_wait_busy_clear(spi_inst_t *spi, uint8_t cs_pin, 
                                  uint32_t timeout_ms, uint32_t *out_elapsed_ms) {
    uint64_t t0 = time_us_64();
    while ((flash_rdsr(spi, cs_pin) & 0x01) != 0) {
        if ((time_us_64() - t0) > (uint64_t)timeout_ms * 1000ull) {
            if (out_elapsed_ms) *out_elapsed_ms = (uint32_t)((time_us_64() - t0) / 1000ull);
            return false;
        }
        sleep_us(200);
    }
    if (out_elapsed_ms) *out_elapsed_ms = (uint32_t)((time_us_64() - t0) / 1000ull);
    return true;
}

static void flash_erase_cmd(spi_inst_t *spi, uint8_t cs_pin, uint8_t op, uint32_t addr) {
    flash_wren(spi, cs_pin);
    if (op == 0xC7 || op == 0x60) {
        uint8_t c = op;
        cs_low(cs_pin);
        spi_tx(spi, &c, 1);
        cs_high(cs_pin);
    } else {
        uint8_t h[4] = {op, (uint8_t)(addr >> 16), (uint8_t)(addr >> 8), (uint8_t)addr};
        cs_low(cs_pin);
        spi_tx(spi, h, 4);
        cs_high(cs_pin);
    }
}

static void flash_read03(spi_inst_t *spi, uint8_t cs_pin, uint32_t addr, uint8_t *buf, size_t len) {
    uint8_t h[4] = {0x03, (uint8_t)(addr >> 16), (uint8_t)(addr >> 8), (uint8_t)addr};
    cs_low(cs_pin);
    spi_tx(spi, h, 4);
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

static void print_erase_header(int mhz) {
    char title[64];
    snprintf(title, sizeof title, "ERASE BENCHMARKS @ %d MHz (times in ms)", mhz);
    print_section(title);
    printf("type       |   n |     avg(ms) | DB_typ | DB_max\n");
    print_divider(60);
}

// Results management
void erase_reset_results(void) {
    memset(&g_erase_result, 0, sizeof(g_erase_result));
}

void erase_save_result(int mhz, double avg4k, uint32_t min4k, uint32_t max4k,
                       double avg32k, uint32_t min32k, uint32_t max32k,
                       double avg64k, uint32_t min64k, uint32_t max64k) {
    g_erase_result.clock_mhz = mhz;
    g_erase_result.valid = true;
    g_erase_result.avg_4k = avg4k;
    g_erase_result.min_4k = min4k;
    g_erase_result.max_4k = max4k;
    g_erase_result.avg_32k = avg32k;
    g_erase_result.min_32k = min32k;
    g_erase_result.max_32k = max32k;
    g_erase_result.avg_64k = avg64k;
    g_erase_result.min_64k = min64k;
    g_erase_result.max_64k = max64k;
}

// Benchmark one erase operation - TIME ENTIRE BATCH
static void bench_one_erase(spi_inst_t *spi, uint8_t cs_pin, const char *label,
                           uint8_t opcode, uint32_t size_bytes,
                           uint32_t base_addr, uint16_t db_typ_ms, uint16_t db_max_ms,
                           int mhz_for_csv_unused,
                           double *out_avg, uint32_t *out_min, uint32_t *out_max) {
    (void)mhz_for_csv_unused;
    
    uint32_t addr = base_addr & ~(size_bytes - 1);
    uint32_t total_ms = 0;
    
    // Time the ENTIRE batch of erase operations
    uint64_t batch_start = time_us_64();
    
    for (int i = 0; i < ITERS_ERASE; i++) {
        flash_erase_cmd(spi, cs_pin, opcode, addr);
        uint32_t elapsed_ms = 0;
        if (!flash_wait_busy_clear(spi, cs_pin, 60000, &elapsed_ms))
            printf("  [WARN] ERASE_TIMEOUT, OP=0x%02X, ADDR=0x%06X\n", opcode, (unsigned)addr);
        
        uint8_t chk[16];
        flash_read03(spi, cs_pin, addr, chk, sizeof chk);
        bool erased = true;
        for (int k = 0; k < 16; k++)
            if (chk[k] != 0xFF) {
                erased = false;
                break;
            }
        if (!erased)
            printf("  [WARN] ERASE_VERIFY_NOT_BLANK, ADDR=0x%06X\n", (unsigned)addr);
    }
    
    uint64_t batch_end = time_us_64();
    total_ms = (uint32_t)((batch_end - batch_start) / 1000);
    
    // Calculate average time per erase
    double avg_ms = (double)total_ms / (double)ITERS_ERASE;
    
    printf("%-10s | %3d | %10.3f | %6u | %6u\n",
           label, ITERS_ERASE, avg_ms, db_typ_ms, db_max_ms);
    print_divider(60);

    if (out_avg) *out_avg = avg_ms;
    if (out_min) *out_min = (uint32_t)avg_ms;
    if (out_max) *out_max = (uint32_t)avg_ms;
}

void erase_run_benches_at_clock(spi_inst_t *spi, uint8_t cs_pin,
                                const erase_ident_t *id,
                                const erase_chip_db_entry_t *chip,
                                int mhz, uint32_t test_addr) {
    uint32_t actual = spi_set_baudrate(spi, (uint32_t)mhz * 1000u * 1000u);
    int mhz_print = (int)(actual / 1000000u);
    print_erase_header(mhz_print);

    bool has4 = false, has32 = false, has64 = false;
    uint8_t o4 = 0, o32 = 0, o64 = 0;
    for (int k = 0; k < 4; k++) {
        if (!id->et_present[k]) continue;
        if (id->et_size_bytes[k] == SECTOR_4K) {
            has4 = true;
            o4 = id->et_opcode[k];
        }
        if (id->et_size_bytes[k] == BLOCK_32K) {
            has32 = true;
            o32 = id->et_opcode[k];
        }
        if (id->et_size_bytes[k] == BLOCK_64K) {
            has64 = true;
            o64 = id->et_opcode[k];
        }
    }
    if (!has4) {
        has4 = true;
        o4 = 0x20;
    }
    if (!has32) {
        has32 = true;
        o32 = 0x52;
    }
    if (!has64) {
        has64 = true;
        o64 = 0xD8;
    }

    uint16_t typ_4k = chip ? chip->typ_4kb_erase_ms : 0;
    uint16_t max_4k = chip ? chip->max_4kb_erase_ms : 0;
    uint16_t typ_32k = chip ? chip->typ_32kb_erase_ms : 0;
    uint16_t max_32k = chip ? chip->max_32kb_erase_ms : 0;
    uint16_t typ_64k = chip ? chip->typ_64kb_erase_ms : 0;
    uint16_t max_64k = chip ? chip->max_64kb_erase_ms : 0;

    // Timing result variables for each erase size
    double avg_4k_time = 0;
    uint32_t min_4k_time = 0, max_4k_time = 0;
    double avg_32k_time = 0;
    uint32_t min_32k_time = 0, max_32k_time = 0;
    double avg_64k_time = 0;
    uint32_t min_64k_time = 0, max_64k_time = 0;

    if (has4) {
        printf("  [TEST] 4K erase with opcode 0x%02X\n", o4);
        bench_one_erase(spi, cs_pin, "4K-erase", o4, SECTOR_4K, test_addr, typ_4k, max_4k,
                       mhz_print, &avg_4k_time, &min_4k_time, &max_4k_time);
    }
    if (has32) {
        printf("  [TEST] 32K erase with opcode 0x%02X\n", o32);
        bench_one_erase(spi, cs_pin, "32K-erase", o32, BLOCK_32K, test_addr, typ_32k, max_32k,
                       mhz_print, &avg_32k_time, &min_32k_time, &max_32k_time);
    }
    if (has64) {
        printf("  [TEST] 64K erase with opcode 0x%02X\n", o64);
        bench_one_erase(spi, cs_pin, "64K-erase", o64, BLOCK_64K, test_addr, typ_64k, max_64k,
                       mhz_print, &avg_64k_time, &min_64k_time, &max_64k_time);
    }

    // Save results
    erase_save_result(mhz_print, avg_4k_time, min_4k_time, max_4k_time,
                     avg_32k_time, min_32k_time, max_32k_time,
                     avg_64k_time, min_64k_time, max_64k_time);
}

void erase_print_summary_tables(void) {
    if (g_erase_result.valid) {
        print_section("ERASE BENCHMARK SUMMARY");
        printf("\n=== ERASE PERFORMANCE SUMMARY (@ %d MHz) ===\n", g_erase_result.clock_mhz);
        printf("Type    |  Avg (ms)  | Min (ms) | Max (ms)\n");
        printf("--------+------------+----------+---------\n");
        printf("4K      | %10.3f | %8u | %7u\n", g_erase_result.avg_4k, g_erase_result.min_4k, g_erase_result.max_4k);
        printf("32K     | %10.3f | %8u | %7u\n", g_erase_result.avg_32k, g_erase_result.min_32k, g_erase_result.max_32k);
        printf("64K     | %10.3f | %8u | %7u\n", g_erase_result.avg_64k, g_erase_result.min_64k, g_erase_result.max_64k);
        printf("--------+------------+----------+---------\n");
    }
}

void erase_flash_unprotect(spi_inst_t *spi, uint8_t cs_pin, uint8_t mfr, uint32_t test_addr) {
    (void)mfr;
    (void)test_addr;
    uint8_t sr1 = flash_rdsr(spi, cs_pin);
    uint8_t sr2 = flash_rdsr2(spi, cs_pin);
    uint8_t new_sr1 = (uint8_t)(sr1 & ~(uint8_t)0x1C);
    flash_wren_sr_volatile(spi, cs_pin);
    flash_wrsr1(spi, cs_pin, new_sr1);
    flash_wrsr2(spi, cs_pin, (uint8_t)(sr2 & ~(1u << 6)));
    flash_wait_busy_clear(spi, cs_pin, 50, NULL);

    uint8_t chk1 = flash_rdsr(spi, cs_pin), chk2 = flash_rdsr2(spi, cs_pin);
    if (chk1 & 0x1C)
        printf("  [WARN] UNPROTECT_PARTIAL, SR1=0x%02X->0x%02X, SR2=0x%02X->0x%02X\n", sr1, chk1, sr2, chk2);
    else
        printf("  [OK]   UNPROTECT, SR1=0x%02X->0x%02X, SR2=0x%02X->0x%02X\n", sr1, chk1, sr2, chk2);
}