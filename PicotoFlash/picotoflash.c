/*
 * Complete Flash Chip Identification System with Integrated Benchmarking
 * Master Pico Module
 *
 * GP20 short press (single action):
 *   1) Identify the flash
 *   2) SAFE WRITE/VERIFY TEST (non-destructive; restores original 256B)
 *   3) **AUTO BACKUP to SD** → /univ_<JEDEC>.bin
 *   4) Run read/write/erase benchmarks
 *   5) Match against database, display + save reports
 *   6) Summaries
 *   7) **AUTO RESTORE from the SD backup just created** (verify on)
 *      → then immediately dump /state_after_restore_<JEDEC>.bin for manual compare
 *
 * GP21 short press: View database
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/stdio.h"
#include "hardware/gpio.h"
#include "hardware/rtc.h"
#include "hardware/spi.h"
#include "hardware/clocks.h"
#include "ff.h"
#include "fatfs/FatFs_SPI/sd_driver/sd_card.h"
#include "identification.h"
#include "sd_functions.h"
#include "display_functions.h"
#include "read.h"
#include "erase.h"
#include "univ_restore_sd.h"
#include "write.h"

// === Universal JEDEC backup/restore module ===
#include "jedec_universal_backup.h"
#pragma once

// ========== Pin Definitions ==========
#define BUTTON_PIN 20              // GP20 - Full automated pipeline
#define DISPLAY_BUTTON_PIN 21      // GP21 - View database
#define FLASH_SPI spi0
#define PIN_SCK 2
#define PIN_MOSI 3
#define PIN_MISO 4
#define PIN_CS 6

// ========== System Constants ==========
#define DEBOUNCE_DELAY_MS 50
#define MAX_MOUNT_ATTEMPTS 3
#define MOUNT_RETRY_DELAY_MS 500
#define POST_MOUNT_DELAY_MS 200
#define TEST_BASE_ADDR 0x100000u   // 1MB offset (safe area)
#define PAGE_SIZE 256u
#define ENABLE_DESTRUCTIVE_TESTS 1

// ========== Global Variables ==========
FlashChipData database[MAX_DATABASE_ENTRIES];
int database_entry_count = 0;
FlashChipData benchmark_results;
match_result_t match_results[TOP_MATCHES_COUNT];
bool database_loaded = false;

// Track last detected JEDEC and last backup path
static uint8_t g_last_jedec[3] = {0};
static char    g_last_backup_path[64] = {0};

// Dynamic test chip data (populated by benchmarks)
FlashChipData test_chip = {
    .chip_model = "UNKNOWN",
    .company = "",
    .chip_family = "",
    .capacity_mbit = 0,
    .jedec_id = "",
    .read_speed_max = 0.0,
    .erase_speed = 0.0,
    .max_clock_freq_mhz = 0,
    .typ_4kb_erase_ms = 0.0,
    .max_4kb_erase_ms = 0.0,
    .typ_32kb_erase_ms = 0.0,
    .max_32kb_erase_ms = 0.0,
    .typ_64kb_erase_ms = 0.0,
    .max_64kb_erase_ms = 0.0,
    .typ_page_program_ms = 0.0,
    .max_page_program_ms = 0.0
};

// ========== Flash SPI Helper Functions ==========
static inline void cs_low(void)  { gpio_put(PIN_CS, 0); }
static inline void cs_high(void) { gpio_put(PIN_CS, 1); }
static inline void spi_tx(const uint8_t *b, size_t n) { spi_write_blocking(FLASH_SPI, b, n); }
static inline void spi_rx(uint8_t *b, size_t n)       { spi_read_blocking(FLASH_SPI, 0x00, b, n); }

// ========== Minimal Read Helpers ==========
static void read_jedec_id(uint8_t out[3]) {
    uint8_t c = 0x9F;
    cs_low(); spi_tx(&c, 1); spi_rx(out, 3); cs_high();
}
static void flash_read_03(uint32_t addr, uint8_t *buf, size_t len) {
    uint8_t h[4] = {0x03, (uint8_t)(addr >> 16), (uint8_t)(addr >> 8), (uint8_t)addr};
    cs_low(); spi_tx(h, 4); spi_rx(buf, len); cs_high();
}

// ========== WRITE TEST LOW-LEVEL HELPERS (non-destructive) ==========
static void flash_write_enable(void) {
    uint8_t cmd = 0x06;
    cs_low(); spi_tx(&cmd, 1); cs_high();
}
static uint8_t flash_read_status1(void) {
    uint8_t cmd = 0x05, v = 0;
    cs_low(); spi_tx(&cmd, 1); spi_rx(&v, 1); cs_high();
    return v;
}
static void flash_wait_busy(void) {
    while (flash_read_status1() & 0x01) sleep_ms(1);
}
static void flash_page_program(uint32_t addr, const uint8_t *buf, size_t len) {
    // assumes len <= 256 and does not cross page boundary
    flash_write_enable();
    uint8_t hdr[4] = {0x02, (uint8_t)(addr >> 16), (uint8_t)(addr >> 8), (uint8_t)addr};
    cs_low(); spi_tx(hdr, 4); spi_tx(buf, len); cs_high();
    flash_wait_busy();
}

// ========== Identification Structure ==========
typedef struct {
    uint8_t jedec[3];
    bool sfdp_ok;
    uint8_t sfdp_major, sfdp_minor;
    uint32_t density_bits;
    bool et_present[4];
    uint8_t et_opcode[4];
    uint32_t et_size_bytes[4];
    bool fastread_0B;
    uint8_t fastread_dummy;
} ident_t;

static bool read_sfdp(uint32_t a, uint8_t *buf, size_t n) {
    if (a > 0xFFFFFF) return false;
    uint8_t h[5] = {0x5A, (uint8_t)(a >> 16), (uint8_t)(a >> 8), (uint8_t)a, 0};
    cs_low(); spi_tx(h, 5); spi_rx(buf, n); cs_high();
    return true;
}

// ==================== JEDEC FALLBACK CAPACITY ====================
static void jedec_fallback_capacity(ident_t *id) {
    uint8_t last_byte = id->jedec[2];  // last JEDEC byte
    switch (last_byte) {
        case 0x18: test_chip.capacity_mbit = 128.0f; break;
        case 0x17: test_chip.capacity_mbit = 64.0f;  break;
        case 0x16: test_chip.capacity_mbit = 32.0f;  break;
        case 0x15: test_chip.capacity_mbit = 16.0f;  break;
        case 0x14: test_chip.capacity_mbit = 8.0f;   break;
        case 0x13: test_chip.capacity_mbit = 4.0f;   break;
        case 0x12: test_chip.capacity_mbit = 2.0f;   break;
        case 0x11: test_chip.capacity_mbit = 1.0f;   break;
        case 0x10: test_chip.capacity_mbit = 0.5f;   break;
        default:   test_chip.capacity_mbit = 0.0f;   break;
    }
    printf("[FALLBACK] Using JEDEC fallback capacity: %.3f Mbit\n", test_chip.capacity_mbit);
}

static void identify(ident_t *id) {
    memset(id, 0, sizeof(*id));
    read_jedec_id(id->jedec);

    // Save and set lower SPI speed for SFDP
    uint32_t saved = spi_get_baudrate(FLASH_SPI);
    spi_set_baudrate(FLASH_SPI, 5 * 100 * 1000);

    // Read SFDP header
    uint8_t hdr[8] = {0};
    if (read_sfdp(0, hdr, 8) && hdr[0] == 'S' && hdr[1] == 'F' && hdr[2] == 'D' && hdr[3] == 'P') {
        id->sfdp_ok = true;
        id->sfdp_minor = hdr[4];
        id->sfdp_major = hdr[5];

        uint8_t nph = (uint8_t)(hdr[6]) + 1;
        uint8_t ph[8 * 16] = {0};
        size_t need = (size_t)nph * 8;
        if (need > sizeof(ph)) need = sizeof(ph);
        read_sfdp(8, ph, need);

        // Find Basic Flash Parameter Table
        bool bfpt = false;
        uint32_t ptp = 0;
        uint8_t dwords = 0;
        for (uint8_t i = 0; i < nph; i++) {
            const uint8_t *p = &ph[i * 8];
            uint16_t idlsb = ((uint16_t)p[0]) | (((uint16_t)p[1]) << 8);
            if (idlsb == 0xFF00) {
                idlsb = 0x00FF;
                bfpt = true;
                ptp = ((uint32_t)p[5]) | (((uint32_t)p[6]) << 8) | (((uint32_t)p[7]) << 16);
                dwords = p[4];
                break;
            }
        }

        if (!bfpt) {
            ptp = 0x000030;
            dwords = 64;
            bfpt = true;
        }

        if (bfpt) {
            size_t bytes = (size_t)(dwords * 4);
            if (bytes > 256) bytes = 256;
            uint8_t bf[256] = {0};
            read_sfdp(ptp, bf, bytes);

            // Parse density (DWORD 2)
            if (bytes >= 8) {
                uint32_t d2 = ((uint32_t)bf[4]) | (((uint32_t)bf[5]) << 8) |
                              (((uint32_t)bf[6]) << 16) | (((uint32_t)bf[7]) << 24);
                if ((d2 & 0x80000000u) == 0) {
                    id->density_bits = d2 + 1u;
                } else {
                    uint32_t n = (d2 & 0x7FFFFFFFu) + 1u;
                    if (n >= 32) id->density_bits = (1u << n);
                }
            }

            // Parse erase types (DWORD 7 & 8)
            if (bytes >= 32) {
                uint32_t d7 = ((uint32_t)bf[24]) | (((uint32_t)bf[25]) << 8) |
                              (((uint32_t)bf[26]) << 16) | (((uint32_t)bf[27]) << 24);
                uint32_t d8 = ((uint32_t)bf[28]) | (((uint32_t)bf[29]) << 8) |
                              (((uint32_t)bf[30]) << 16) | (((uint32_t)bf[31]) << 24);

                uint8_t szn[4] = {(uint8_t)(d7 >> 0), (uint8_t)(d7 >> 16),
                                  (uint8_t)(d8 >> 0), (uint8_t)(d8 >> 16)};
                uint8_t opc[4] = {(uint8_t)(d7 >> 8), (uint8_t)(d7 >> 24),
                                  (uint8_t)(d8 >> 8), (uint8_t)(d8 >> 24)};

                for (int k = 0; k < 4; k++) {
                    uint32_t sz = (1u << szn[k]);
                    id->et_present[k] = (sz != 0);
                    id->et_opcode[k] = opc[k];
                    id->et_size_bytes[k] = sz;
                }
            }
        }
    }

    // Light probe: try 0x0B
    uint8_t c[5] = {0x0B, 0, 0, 0, 0}, v = 0xA5;
    cs_low(); spi_tx(c, 5); spi_rx(&v, 1); cs_high();
    id->fastread_0B = true; id->fastread_dummy = 1;

    // restore SPI baud
    spi_set_baudrate(FLASH_SPI, saved);
}

// ========== Benchmark Data Capture ==========
static void populate_test_chip_from_identification(ident_t *id) {
    snprintf(test_chip.jedec_id, sizeof(test_chip.jedec_id),
             "%02X %02X %02X", id->jedec[0], id->jedec[1], id->jedec[2]);

    if (id->density_bits == 0 || id->density_bits < 1024) {
        jedec_fallback_capacity(id);
    } else {
        test_chip.capacity_mbit = (float)(id->density_bits) / 1048576.0f;
    }

    strcpy(test_chip.chip_model, "UNKNOWN");
    strcpy(test_chip.company, "");
    strcpy(test_chip.chip_family, "");

    // store for restore filename
    memcpy(g_last_jedec, id->jedec, 3);

    printf("\n");
    printf("=======================================================\n");
    printf(" CHIP IDENTIFICATION\n");
    printf("=======================================================\n");
    printf("JEDEC ID:     %s\n", test_chip.jedec_id);
    printf("Capacity:     %.3f Mbit\n", test_chip.capacity_mbit);
    printf("SFDP Version: %u.%u\n", id->sfdp_major, id->sfdp_minor);
    printf("=======================================================\n");
}

static void capture_read_benchmark_results(void) {
    double speed_50mhz = read_get_50mhz_speed();
    if (speed_50mhz > 0.0) {
        test_chip.read_speed_max = (float)speed_50mhz;
        printf("\n[CAPTURE] Read speed at 50MHz: %.2f MB/s\n", test_chip.read_speed_max);
    } else {
        test_chip.read_speed_max = 0.0;
        printf("\n[WARNING] Could not derive 50MHz read speed\n");
    }
}

static void capture_erase_benchmark_results(void) {
    erase_result_t erase_data = erase_get_results();

    if (erase_data.valid) {
        test_chip.typ_4kb_erase_ms = (float)erase_data.avg_4k;
        test_chip.max_4kb_erase_ms = (float)erase_data.max_4k;
        test_chip.typ_32kb_erase_ms = (float)erase_data.avg_32k;
        test_chip.max_32kb_erase_ms = (float)erase_data.max_32k;
        test_chip.typ_64kb_erase_ms = (float)erase_data.avg_64k;
        test_chip.max_64kb_erase_ms = (float)erase_data.max_64k;
        test_chip.erase_speed = test_chip.typ_64kb_erase_ms;

        printf("\n");
        printf("=======================================================\n");
        printf(" ERASE TIMING CAPTURE\n");
        printf("=======================================================\n");
        printf("4KB  Erase: avg=%.1f ms, max=%.1f ms\n",
               test_chip.typ_4kb_erase_ms, test_chip.max_4kb_erase_ms);
        printf("32KB Erase: avg=%.1f ms, max=%.1f ms\n",
               test_chip.typ_32kb_erase_ms, test_chip.max_32kb_erase_ms);
        printf("64KB Erase: avg=%.1f ms, max=%.1f ms\n",
               test_chip.typ_64kb_erase_ms, test_chip.max_64kb_erase_ms);
        printf("=======================================================\n");
    } else {
        printf("\n[WARNING] Erase benchmark results not available\n");
    }
}

// === AUTO BACKUP to SD right after identification ===
typedef struct { FIL file; uint64_t written; } sd_sink_ctx_t;

static bool sd_sink(const uint8_t* data, size_t len, uint32_t off, void* user) {
    (void)off;
    sd_sink_ctx_t* ctx = (sd_sink_ctx_t*)user;
    UINT bw = 0;
    FRESULT fr = f_write(&ctx->file, data, (UINT)len, &bw);
    if (fr != FR_OK || bw != len) return false;
    ctx->written += bw;
    if ((ctx->written & ((1u<<20)-1u)) == 0) {
        printf("[UNIV] %llu bytes...\n", (unsigned long long)ctx->written);
    }
    return true;
}

// returns true on success and stores absolute path in outpath
static bool universal_dump_after_ident(char *outpath, size_t outsz) {
    if (!outpath || outsz < 8) return false;

    jedec_bus_t bus = {
        .spi = FLASH_SPI,
        .cs_pin = PIN_CS,
        .wp_pin = 0xFFFFFFFF,
        .hold_pin = 0xFFFFFFFF,
        .sck_pin = PIN_SCK,
        .mosi_pin = PIN_MOSI,
        .miso_pin = PIN_MISO,
        .clk_hz = 16000000
    };
    jedec_chip_t chip;
    jedec_init(&bus);
    jedec_probe(&chip);

    printf("[UNIV] JEDEC %02X %02X %02X  size=%u 4B=%d cmd=0x%02X\n",
           chip.manuf_id, chip.mem_type, chip.capacity_id,
           chip.total_bytes, chip.use_4byte_addr, chip.read_cmd);

    // filename by JEDEC
    snprintf(outpath, outsz, "/univ_%02X%02X%02X.bin",
             chip.manuf_id, chip.mem_type, chip.capacity_id);

    sd_sink_ctx_t ctx = {0};
    FRESULT fr = f_open(&ctx.file, outpath, FA_CREATE_ALWAYS | FA_WRITE);
    if (fr != FR_OK) {
        printf("[UNIV] SD open failed (%d) for %s\n", fr, outpath);
        outpath[0] = '\0';
        return false;
    }

    printf("[UNIV] Backing up %u bytes to %s...\n", chip.total_bytes, outpath);
    bool ok = jedec_backup_full(&chip, sd_sink, &ctx);
    f_close(&ctx.file);

    printf("[UNIV] %s, wrote %llu bytes\n",
           ok ? "DONE" : "ERROR/ABORT", (unsigned long long)ctx.written);
    return ok;
}

/* =======================
 * POST-RESTORE SNAPSHOT
 * Dump current flash to SD as /state_after_restore_<JEDEC>.bin
 * Small 8 KB chunks to avoid RAM pressure.
 * ======================= */
// ---- robust post-restore dumper using jedec_backup_stream ----
typedef struct { FIL file; uint64_t written; uint32_t last_err; } postdump_ctx_t;

static bool postdump_sink(const uint8_t* data, size_t len, uint32_t off, void* user) {
    (void)off;
    postdump_ctx_t* ctx = (postdump_ctx_t*)user;
    UINT bw = 0;
    FRESULT fr = f_write(&ctx->file, data, (UINT)len, &bw);
    if (fr != FR_OK || bw != len) {
        ctx->last_err = fr ? fr : 0xFFFF0000u | (UINT)bw; // stash something diagnostic
        printf("[POSTDUMP] f_write err=%d bw=%u len=%u\n", (int)fr, (unsigned)bw, (unsigned)len);
        return false;
    }
    ctx->written += bw;

    // Flush every ~1 MiB so size updates even if we crash later
    if ((ctx->written & ((1u<<20)-1u)) == 0) {
        f_sync(&ctx->file);
        printf("[POSTDUMP] %llu bytes...\n", (unsigned long long)ctx->written);
    }
    return true;
}

// --- Post-restore snapshot using the SAME flow as the working backup ---
static bool make_state_after_restore_dump(void) {
    // Reuse the same bus config used by backup
    jedec_bus_t bus = {
        .spi = FLASH_SPI,
        .cs_pin = PIN_CS,
        .wp_pin = 0xFFFFFFFF,
        .hold_pin = 0xFFFFFFFF,
        .sck_pin = PIN_SCK,
        .mosi_pin = PIN_MOSI,
        .miso_pin = PIN_MISO,
        .clk_hz = 16000000
    };

    jedec_chip_t chip;
    jedec_init(&bus);
    jedec_probe(&chip);

    char path[64];
    // Use the SAME path style as your working backup (no drive prefix)
    // If your backup used "0:/univ_..." then change this to "0:/state_after_restore_...".
    snprintf(path, sizeof(path), "/state_after_restore_%02X%02X%02X.bin",
             chip.manuf_id, chip.mem_type, chip.capacity_id);

    sd_sink_ctx_t ctx = {0};
    FRESULT fr = f_open(&ctx.file, path, FA_CREATE_ALWAYS | FA_WRITE);
    if (fr != FR_OK) {
        printf("[POSTDUMP] open failed (%d) for %s\n", (int)fr, path);
        return false;
    }

    printf("[POSTDUMP] Dumping %u bytes to %s ...\n", chip.total_bytes, path);

    bool ok = jedec_backup_full(&chip, sd_sink, &ctx);

    f_sync(&ctx.file);  // optional, but nice to see size update live
    f_close(&ctx.file);

    printf("[POSTDUMP] %s — wrote %llu bytes\n",
           ok ? "DONE" : "ERROR/ABORT", (unsigned long long)ctx.written);
    return ok;
}


// ========== Main Function ==========
int main(void) {
    stdio_init_all();
    sleep_ms(2000);

    printf("\n");
    printf("===============================================\n");
    printf(" UNIFIED FLASH PIPELINE (AUTO RESTORE AT END)\n");
    printf("===============================================\n");
    printf("System clock: %u Hz\n", (unsigned)clock_get_hz(clk_sys));
    printf("Peripheral clock: %u Hz\n", (unsigned)clock_get_hz(clk_peri));
    printf("\n");

    // Initialize RTC
    datetime_t t = {.year=2024,.month=1,.day=1,.dotw=1,.hour=0,.min=0,.sec=0};
    rtc_init(); rtc_set_datetime(&t);

    // Initialize SPI for flash
    spi_init(FLASH_SPI, 5 * 100 * 1000);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_init(PIN_CS); gpio_set_dir(PIN_CS, GPIO_OUT); cs_high();

    // Initialize buttons
    gpio_init(BUTTON_PIN); gpio_set_dir(BUTTON_PIN, GPIO_IN); gpio_pull_up(BUTTON_PIN);
    gpio_init(DISPLAY_BUTTON_PIN); gpio_set_dir(DISPLAY_BUTTON_PIN, GPIO_IN); gpio_pull_up(DISPLAY_BUTTON_PIN);

    display_system_banner();

    // Initialize SD card
    FATFS fs;
    bool sd_mounted = false;
    int mount_attempts = 0;

    while (!sd_mounted && mount_attempts < MAX_MOUNT_ATTEMPTS) {
        display_sd_mount_attempt(mount_attempts + 1, MAX_MOUNT_ATTEMPTS);
        FRESULT fr = f_mount(&fs, "0:", 1);
        if (fr == FR_OK) {
            sd_mounted = true;
            display_sd_mount_success();
            display_sd_stabilization();
            sleep_ms(POST_MOUNT_DELAY_MS);
        } else {
            display_sd_mount_warning(fr);
            mount_attempts++;
            if (mount_attempts < MAX_MOUNT_ATTEMPTS) sleep_ms(MOUNT_RETRY_DELAY_MS);
        }
    }

    if (!sd_mounted) {
        display_sd_mount_failed(MAX_MOUNT_ATTEMPTS);
    } else {
        int load_result = sd_load_chip_database();
        if (load_result == SUCCESS) {
            database_loaded = true;
            display_database_loaded(database_entry_count);
        }
    }

    display_startup_instructions();

    printf("\nGP20: run full flow (auto-restore at end)\n");
    printf("GP21: view database\n\n");

    // Button state tracking
    bool last_button_state = true;
    bool last_display_button_state = true;
    uint32_t last_button_time = 0;
    uint32_t last_display_button_time = 0;

    while (true) {
        bool current_button_state = gpio_get(BUTTON_PIN);
        bool current_display_button_state = gpio_get(DISPLAY_BUTTON_PIN);
        uint32_t current_time = to_ms_since_boot(get_absolute_time());

        // ==================== GP21 BUTTON - VIEW DATABASE ====================
        if (last_display_button_state && !current_display_button_state &&
            (current_time - last_display_button_time) > DEBOUNCE_DELAY_MS) {

            display_button_pressed_gp21();

            if (!sd_mounted) {
                mount_attempts = 0;
                while (!sd_mounted && mount_attempts < MAX_MOUNT_ATTEMPTS) {
                    display_sd_mount_attempt(mount_attempts + 1, MAX_MOUNT_ATTEMPTS);
                    FRESULT fr = f_mount(&fs, "0:", 1);
                    if (fr == FR_OK) {
                        sd_mounted = true;
                        display_sd_mount_success();
                        display_sd_stabilization();
                        sleep_ms(POST_MOUNT_DELAY_MS);
                        int load_result = sd_load_chip_database();
                        if (load_result == SUCCESS) {
                            database_loaded = true;
                            display_database_loaded(database_entry_count);
                        }
                    } else {
                        display_sd_mount_warning(fr);
                        mount_attempts++;
                        if (mount_attempts < MAX_MOUNT_ATTEMPTS) sleep_ms(MOUNT_RETRY_DELAY_MS);
                    }
                }
                if (!sd_mounted) {
                    printf("ERROR: SD card not mounted after %d attempts\n", MAX_MOUNT_ATTEMPTS);
                    last_display_button_time = current_time;
                    last_display_button_state = current_display_button_state;
                    continue;
                }
            }

            display_full_database();
            last_display_button_time = current_time;
        }
        last_display_button_state = current_display_button_state;

        // ==================== GP20 BUTTON - FULL FLOW (auto restore at end) ====================
        if (last_button_state && !current_button_state &&
            (current_time - last_button_time) > DEBOUNCE_DELAY_MS) {

            printf("\n*******************************************************\n");
            printf(" BUTTON PRESSED - STARTING FULL FLOW\n");
            printf("*******************************************************\n");
            sleep_ms(100);

            // Reset benchmark results
            read_reset_results();
            erase_reset_results();

            // Reset test_chip data
            memset(&test_chip, 0, sizeof(test_chip));
            strcpy(test_chip.chip_model, "UNKNOWN");
            g_last_backup_path[0] = '\0';

            // ===== STEP 1: IDENTIFY CHIP =====
            printf("\n[STEP 1/7] Identifying Flash Chip...\n");
            ident_t id; memset(&id, 0, sizeof(id));
            identify(&id);
            populate_test_chip_from_identification(&id);

            // ===== STEP 2: SAFE WRITE/VERIFY TEST (non-destructive) =====
            printf("\n[STEP 2/7] Write/Verify Test (non-destructive)...\n");
            {
                const uint32_t TEST_ADDR = 0x00010000; // 64KB offset (should be safe)
                uint8_t original[256], pattern[256], verify[256];

                flash_read_03(TEST_ADDR, original, 256);
                for (int i = 0; i < 256; i++) pattern[i] = (uint8_t)(i ^ 0xA5);

                flash_page_program(TEST_ADDR, pattern, 256);
                flash_read_03(TEST_ADDR, verify, 256);

                bool ok = true;
                for (int i = 0; i < 256; i++) { if (verify[i] != pattern[i]) { ok = false; break; } }
                printf("[WRITE TEST] %s\n", ok ? "✅ SUCCESS — write + verify OK" : "❌ FAILED — data mismatch");

                flash_page_program(TEST_ADDR, original, 256);
                printf("[WRITE TEST] Original data restored.\n");
            }

            // ===== STEP 3: AUTO BACKUP TO SD =====
            printf("\n[STEP 3/7] Auto backup to SD (pre-benchmarks)...\n");
            bool backup_ok = false;
            if (sd_mounted) {
                backup_ok = universal_dump_after_ident(g_last_backup_path, sizeof(g_last_backup_path));
                if (!backup_ok) printf("[AUTO BACKUP] Failed. Continuing with benchmarks.\n");
            } else {
                printf("[AUTO BACKUP] Skipped (SD not mounted).\n");
            }

            // ===== STEP 4: READ BENCHMARKS =====
            printf("\n[STEP 4/7] Running Read Benchmarks...\n");
            {
                bool use_fast = id.fastread_0B;
                uint8_t dummy = use_fast ? (id.fastread_dummy ? id.fastread_dummy : 1) : 0;
                const int clock_list[] = {63, 32, 21, 16, 13};
                const int NCLK = (int)(sizeof(clock_list) / sizeof(clock_list[0]));
                read_bench_capture_t caps[NCLK]; memset(caps, 0, sizeof(caps));

                for (int i = 0; i < NCLK; i++) {
                    int mhz = clock_list[i];
                    printf("  Testing at %d MHz (mode=%s, dummy=%u)\n",
                           mhz, use_fast ? "0x0B" : "0x03", dummy);
                    read_run_benches_capture(FLASH_SPI, PIN_CS, use_fast, dummy, mhz, &caps[i]);
                }
                read_derive_and_print_50(clock_list, caps, NCLK);
                capture_read_benchmark_results();
            }

            // ===== STEP 5: WRITE + ERASE BENCHMARKS =====
#if ENABLE_DESTRUCTIVE_TESTS
            printf("\n[STEP 5/7] Write & Erase Benchmarks...\n");

            // Write benches (summary only; page timing disabled)
            {
                const int write_clocks[] = {21, 16};
                const int num_write_clocks = (int)(sizeof(write_clocks) / sizeof(write_clocks[0]));
                write_bench_capture_t write_captures[num_write_clocks];
                int write_success = write_bench_run_multi_clock(
                    FLASH_SPI, PIN_CS, write_clocks, num_write_clocks,
                    TEST_BASE_ADDR + 0x10000, write_captures);
                if (write_success > 0) write_bench_print_summary(write_captures, num_write_clocks);
                test_chip.typ_page_program_ms = 0.0;
                test_chip.max_page_program_ms = 0.0;
            }

            // Erase benches
            {
                erase_ident_t erase_id;
                memcpy(erase_id.jedec, id.jedec, 3);
                erase_id.sfdp_ok = id.sfdp_ok;
                erase_id.sfdp_major = id.sfdp_major;
                erase_id.sfdp_minor = id.sfdp_minor;
                erase_id.density_bits = id.density_bits;
                memcpy(erase_id.et_present, id.et_present, sizeof(id.et_present));
                memcpy(erase_id.et_opcode, id.et_opcode, sizeof(id.et_opcode));
                memcpy(erase_id.et_size_bytes, id.et_size_bytes, sizeof(id.et_size_bytes));
                erase_id.fast_read_0B = id.fastread_0B;
                erase_id.fast_read_dummy = id.fastread_dummy;

                erase_flash_unprotect(FLASH_SPI, PIN_CS, id.jedec[0], TEST_BASE_ADDR);
                const int ERASE_FIXED_MHZ = 21;
                erase_run_benches_at_clock(FLASH_SPI, PIN_CS, &erase_id, NULL,
                                           ERASE_FIXED_MHZ, TEST_BASE_ADDR);
                capture_erase_benchmark_results();
            }
#else
            printf("\n[STEP 5/7] WRITE/ERASE BENCHMARKS DISABLED\n");
#endif

            // ===== STEP 6: MATCH AGAINST DATABASE =====
            printf("\n[STEP 6/7] Matching Against Database...\n");

            // Ensure DB mounted/loaded
            if (sd_mounted && !database_loaded) {
                display_database_reload_attempt();
                int load_result = sd_load_chip_database();
                if (load_result == SUCCESS) {
                    database_loaded = true;
                    display_database_loaded(database_entry_count);
                } else if (load_result == ERROR_DATABASE_CORRUPT) {
                    display_database_corrupt_warning();
                    f_unmount("0:");
                    sd_mounted = false;
                    database_loaded = false;
                    sleep_ms(100);
                }
            }

            if (!sd_mounted) {
                mount_attempts = 0;
                while (!sd_mounted && mount_attempts < MAX_MOUNT_ATTEMPTS) {
                    display_sd_mount_attempt(mount_attempts + 1, MAX_MOUNT_ATTEMPTS);
                    FRESULT fr = f_mount(&fs, "0:", 1);
                    if (fr == FR_OK) {
                        sd_mounted = true;
                        display_sd_mount_success();
                        display_sd_stabilization();
                        sleep_ms(POST_MOUNT_DELAY_MS);
                        int load_result = sd_load_chip_database();
                        if (load_result == SUCCESS) {
                            database_loaded = true;
                            display_database_loaded(database_entry_count);
                        }
                    } else {
                        display_sd_mount_warning(fr);
                        mount_attempts++;
                        if (mount_attempts < MAX_MOUNT_ATTEMPTS) sleep_ms(MOUNT_RETRY_DELAY_MS);
                    }
                }
                if (!sd_mounted) {
                    printf("ERROR: SD card not mounted after %d attempts\n", MAX_MOUNT_ATTEMPTS);
                }
            }

            if (sd_mounted && database_loaded && database_entry_count > 0) {
                match_status_t status = chip_match_database(&test_chip);
                display_detailed_comparison();
                if (status != MATCH_UNKNOWN) benchmark_results = match_results[0].chip_data;
                sd_log_benchmark_results();
                sd_create_forensic_report();
                display_identification_complete();
            } else {
                display_no_database_error();
            }

            printf("\n*******************************************************\n");
            printf(" SUMMARY\n");
            printf("*******************************************************\n");
            printf("Test Chip Summary:\n");
            printf("  JEDEC ID:          %s\n", test_chip.jedec_id);
            printf("  Capacity:          %.2f Mbit\n", test_chip.capacity_mbit);
            printf("  Read Speed 50MHz:  %.2f MB/s\n", test_chip.read_speed_max);
            printf("  4KB Erase (avg):   %.1f ms\n", test_chip.typ_4kb_erase_ms);
            printf("  32KB Erase (avg):  %.1f ms\n", test_chip.typ_32kb_erase_ms);
            printf("  64KB Erase (avg):  %.1f ms\n", test_chip.typ_64kb_erase_ms);
            printf("*******************************************************\n");

            // ===== STEP 7: AUTO RESTORE FROM SD BACKUP JUST CREATED =====
            printf("\n[STEP 7/7] Auto-restore from SD backup...\n");
if (sd_mounted && g_last_backup_path[0]) {
    jedec_bus_t bus = {
        .spi = FLASH_SPI,
        .cs_pin = PIN_CS,
        .wp_pin = 0xFFFFFFFF,
        .hold_pin = 0xFFFFFFFF,
        .sck_pin = PIN_SCK,
        .mosi_pin = PIN_MOSI,
        .miso_pin = PIN_MISO,
        .clk_hz = 16000000
    };
    printf("[RESTORE] Path: %s\n", g_last_backup_path);
    bool ok = universal_restore_from_sd(g_last_backup_path, &bus, /*verify=*/true);
    printf("[RESTORE] %s\n", ok ? "SUCCESS (verified)" : "FAILED");

    // Optional: keep your state_after_restore dump if you like:
    make_state_after_restore_dump();
} else {
    printf("[RESTORE] Skipped (no SD or no backup path).\n");
}



            printf("\n*******************************************************\n");
            printf(" FULL FLOW COMPLETE (backup+benches+match+restore+postdump)\n");
            printf("*******************************************************\n");

            last_button_time = current_time;
        }
        last_button_state = current_button_state;

        sleep_ms(10);
    }

    return 0;
}
