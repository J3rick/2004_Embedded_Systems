/*
 * Complete Flash Chip Identification System with Integrated Benchmarking
 * Master Pico Module
 * 
 * When GP20 is pressed:
 * 1. Runs comprehensive read/write/erase benchmarks
 * 2. Captures all performance data dynamically
 * 3. Matches against database
 * 4. Displays results and saves to SD card
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>

#include "pico/stdlib.h"
#include "pico/stdio.h"
#include "pico/cyw43_arch.h"

#include "lwip/ip4_addr.h"
#include "lwip/tcp.h"
#include "lwip/pbuf.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"

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
#include "write.h"

// ========== Pin Definitions ==========
#define BUTTON_PIN 20              // GP20 - Run benchmarks & identification
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
static inline void cs_low(void) {
    gpio_put(PIN_CS, 0);
}

static inline void cs_high(void) {
    gpio_put(PIN_CS, 1);
}

static inline void spi_tx(const uint8_t *b, size_t n) {
    spi_write_blocking(FLASH_SPI, b, n);
}

static inline void spi_rx(uint8_t *b, size_t n) {
    spi_read_blocking(FLASH_SPI, 0x00, b, n);
}

// ========== Flash Basic Operations ==========
static void read_jedec_id(uint8_t out[3]) {
    uint8_t c = 0x9F;
    cs_low();
    spi_tx(&c, 1);
    spi_rx(out, 3);
    cs_high();
}

static void flash_read_03(uint32_t addr, uint8_t *buf, size_t len) {
    uint8_t h[4] = {0x03, (uint8_t)(addr >> 16), (uint8_t)(addr >> 8), (uint8_t)addr};
    cs_low();
    spi_tx(h, 4);
    spi_rx(buf, len);
    cs_high();
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
    cs_low();
    spi_tx(h, 5);
    spi_rx(buf, n);
    cs_high();
    return true;
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
    
    // Test if 0x0B Fast Read works
    uint8_t c[5] = {0x0B, 0, 0, 0, 0}, v = 0xA5;
    cs_low();
    spi_tx(c, 5);
    spi_rx(&v, 1);
    cs_high();
    id->fastread_0B = true;
    id->fastread_dummy = 1;
    
    spi_set_baudrate(FLASH_SPI, saved);
}

// ========== Benchmark Data Capture Functions ==========
static void populate_test_chip_from_identification(ident_t *id) {
    // Convert JEDEC ID to string format (matches CSV format: "EF 40 18")
    snprintf(test_chip.jedec_id, sizeof(test_chip.jedec_id), 
             "%02X %02X %02X", id->jedec[0], id->jedec[1], id->jedec[2]);
    
    // Calculate capacity from SFDP density
    if (id->density_bits > 0) {
        // density_bits is in bits, convert to Mbit
        test_chip.capacity_mbit = (float)(id->density_bits) / 1048576.0f;
    }
    
    // Initialize chip info as unknown (will be filled by database match)
    strcpy(test_chip.chip_model, "UNKNOWN");
    strcpy(test_chip.company, "");
    strcpy(test_chip.chip_family, "");
    
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
    // Extract the derived 50MHz read speed
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
    // Extract erase timing results
    erase_result_t erase_data = erase_get_results();
    
    if (erase_data.valid) {
        test_chip.typ_4kb_erase_ms = (float)erase_data.avg_4k;
        test_chip.max_4kb_erase_ms = (float)erase_data.max_4k;
        test_chip.typ_32kb_erase_ms = (float)erase_data.avg_32k;
        test_chip.max_32kb_erase_ms = (float)erase_data.max_32k;
        test_chip.typ_64kb_erase_ms = (float)erase_data.avg_64k;
        test_chip.max_64kb_erase_ms = (float)erase_data.max_64k;
        
        // Calculate erase_speed (using typ_64kb_erase as reference)
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

// ========== Main Function ==========
int main(void) {
    stdio_init_all();
    cyw43_arch_init();
    sleep_ms(2000);

    printf("\n");
    printf("===============================================\n");
    printf(" UNIFIED FLASH BENCHMARK & IDENTIFICATION\n");
    printf("===============================================\n");
    printf("System clock: %u Hz\n", (unsigned)clock_get_hz(clk_sys));
    printf("Peripheral clock: %u Hz\n", (unsigned)clock_get_hz(clk_peri));
    printf("\n");
    
    // Initialize RTC
    datetime_t t = {
        .year = 2024,
        .month = 1,
        .day = 1,
        .dotw = 1,
        .hour = 0,
        .min = 0,
        .sec = 0
    };
    rtc_init();
    rtc_set_datetime(&t);
    cyw43_arch_enable_sta_mode();
    
    // Initialize SPI for flash
    spi_init(FLASH_SPI, 5 * 100 * 1000);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    cs_high();
    
    // Initialize buttons
    gpio_init(BUTTON_PIN);
    gpio_set_dir(BUTTON_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_PIN);
    
    gpio_init(DISPLAY_BUTTON_PIN);
    gpio_set_dir(DISPLAY_BUTTON_PIN, GPIO_IN);
    gpio_pull_up(DISPLAY_BUTTON_PIN);
    
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
            if (mount_attempts < MAX_MOUNT_ATTEMPTS) {
                sleep_ms(MOUNT_RETRY_DELAY_MS);
            }
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
    
    printf("\nGP20 button to start benchmarking & identification...\n");
    printf("GP21 button to view database...\n");
    printf("\n");
    
    // Button state tracking
    bool last_button_state = true;
    bool last_display_button_state = true;
    uint32_t last_button_time = 0;
    uint32_t last_display_button_time = 0;
    
    // Main loop
    while (true) {
        bool current_button_state = gpio_get(BUTTON_PIN);
        bool current_display_button_state = gpio_get(DISPLAY_BUTTON_PIN);
        uint32_t current_time = to_ms_since_boot(get_absolute_time());
        
        // ==================== GP20 BUTTON - RUN BENCHMARKS & IDENTIFICATION ====================
        if (last_button_state && !current_button_state && 
            (current_time - last_button_time) > DEBOUNCE_DELAY_MS) {
            
            printf("\n");
            printf("*******************************************************\n");
            printf(" BUTTON PRESSED - STARTING BENCHMARKS\n");
            printf("*******************************************************\n");
            sleep_ms(100);
            
            // Reset all benchmark results
            read_reset_results();
            erase_reset_results();
            
            // Reset test_chip data
            memset(&test_chip, 0, sizeof(test_chip));
            strcpy(test_chip.chip_model, "UNKNOWN");
            
            // ===== STEP 1: IDENTIFY CHIP =====
            printf("\n[STEP 1/5] Identifying Flash Chip...\n");
            ident_t id;
            memset(&id, 0, sizeof(id));
            identify(&id);
            populate_test_chip_from_identification(&id);
            
            // ===== STEP 2: RUN READ BENCHMARKS =====
            printf("\n[STEP 2/5] Running Read Benchmarks...\n");
            
            bool use_fast = id.fastread_0B;
            uint8_t dummy = use_fast ? (id.fastread_dummy ? id.fastread_dummy : 1) : 0;
            
            const int clock_list[] = {63, 32, 21, 16, 13};
            const int NCLK = (int)(sizeof(clock_list) / sizeof(clock_list[0]));
            read_bench_capture_t caps[NCLK];
            memset(caps, 0, sizeof(caps));
            
            for (int i = 0; i < NCLK; i++) {
                int mhz = clock_list[i];
                printf("  Testing at %d MHz (mode=%s, dummy=%u)\n", 
                       mhz, use_fast ? "0x0B" : "0x03", dummy);
                read_run_benches_capture(FLASH_SPI, PIN_CS, use_fast, dummy, mhz, &caps[i]);
            }
            
            // Derive 50MHz read speed
            read_derive_and_print_50(clock_list, caps, NCLK);
            capture_read_benchmark_results();
            
            // ===== STEP 3: RUN WRITE BENCHMARKS (SKIP PAGE PROGRAM CAPTURE) =====
#if ENABLE_DESTRUCTIVE_TESTS
            printf("\n[STEP 3/5] Running Write Benchmarks...\n");
            printf("NOTE: Page program timing capture is disabled (skipped)\n");
            
            const int write_clocks[] = {21, 16};
            const int num_write_clocks = sizeof(write_clocks) / sizeof(write_clocks[0]);
            write_bench_capture_t write_captures[num_write_clocks];
            
            int write_success = write_bench_run_multi_clock(
                FLASH_SPI, PIN_CS, write_clocks, num_write_clocks, 
                TEST_BASE_ADDR + 0x10000, write_captures);
            
            if (write_success > 0) {
                write_bench_print_summary(write_captures, num_write_clocks);
            }
            
            // Skip page program timing capture (as requested)
            test_chip.typ_page_program_ms = 0.0;
            test_chip.max_page_program_ms = 0.0;
#else
            printf("\n[STEP 3/5] WRITE BENCHMARKS DISABLED\n");
#endif
            
            // ===== STEP 4: RUN ERASE BENCHMARKS =====
#if ENABLE_DESTRUCTIVE_TESTS
            printf("\n[STEP 4/5] Running Erase Benchmarks...\n");
            
            // Convert ident_t to erase_ident_t
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
            
            // Flash unprotect
            erase_flash_unprotect(FLASH_SPI, PIN_CS, id.jedec[0], TEST_BASE_ADDR);
            
            const int ERASE_FIXED_MHZ = 21;
            erase_run_benches_at_clock(FLASH_SPI, PIN_CS, &erase_id, NULL, 
                                       ERASE_FIXED_MHZ, TEST_BASE_ADDR);
            
            capture_erase_benchmark_results();
#else
            printf("\n[STEP 4/5] ERASE BENCHMARKS DISABLED\n");
#endif
            
            // Skip max_clock_freq_mhz (as requested)
            test_chip.max_clock_freq_mhz = 0;
            
            // ===== STEP 5: MATCH AGAINST DATABASE =====
            printf("\n[STEP 5/5] Matching Against Database...\n");
            
            // Check if database needs reloading
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
                    last_button_time = current_time;
                    last_button_state = current_button_state;
                    continue;
                }
            }
            
            // If SD not mounted, try to mount it now
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
                        if (mount_attempts < MAX_MOUNT_ATTEMPTS) {
                            sleep_ms(MOUNT_RETRY_DELAY_MS);
                        }
                    }
                }
                
                if (!sd_mounted) {
                    printf("ERROR: SD card not mounted after %d attempts\n", MAX_MOUNT_ATTEMPTS);
                    last_button_time = current_time;
                    last_button_state = current_button_state;
                    continue;
                }
            }
            
            // Verify database is loaded
            if (!database_loaded || database_entry_count == 0) {
                display_no_database_error();
                last_button_time = current_time;
                last_button_state = current_button_state;
                continue;
            }
            
            // Perform identification
            match_status_t status = chip_match_database(&test_chip);
            
            // Display detailed comparison
            display_detailed_comparison();
            
            // Log results
            if (status != MATCH_UNKNOWN) {
                benchmark_results = match_results[0].chip_data;
            }
            sd_log_benchmark_results();
            
            // Create forensic report
            sd_create_forensic_report();
            
            display_identification_complete();
            
            printf("\n");
            printf("*******************************************************\n");
            printf(" BENCHMARKS & IDENTIFICATION COMPLETE\n");
            printf("*******************************************************\n");
            printf("Test Chip Summary:\n");
            printf("  JEDEC ID:          %s\n", test_chip.jedec_id);
            printf("  Capacity:          %.2f Mbit\n", test_chip.capacity_mbit);
            printf("  Read Speed 50MHz:  %.2f MB/s\n", test_chip.read_speed_max);
            printf("  4KB Erase (avg):   %.1f ms\n", test_chip.typ_4kb_erase_ms);
            printf("  32KB Erase (avg):  %.1f ms\n", test_chip.typ_32kb_erase_ms);
            printf("  64KB Erase (avg):  %.1f ms\n", test_chip.typ_64kb_erase_ms);
            printf("*******************************************************\n");
            printf("GP20 again to rerun benchmarks...\n");
            
            last_button_time = current_time;
        }
        
        last_button_state = current_button_state;
        
        // ==================== GP21 BUTTON - VIEW DATABASE ====================
        if (last_display_button_state && !current_display_button_state && 
            (current_time - last_display_button_time) > DEBOUNCE_DELAY_MS) {
            
            display_button_pressed_gp21();
            
            // If SD not mounted, try to mount it now
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
                        if (mount_attempts < MAX_MOUNT_ATTEMPTS) {
                            sleep_ms(MOUNT_RETRY_DELAY_MS);
                        }
                    }
                }
                
                if (!sd_mounted) {
                    printf("ERROR: SD card not mounted after %d attempts\n", MAX_MOUNT_ATTEMPTS);
                    last_display_button_time = current_time;
                    last_display_button_state = current_display_button_state;
                    continue;
                }
            }
            
            // Display full database
            display_full_database();
            
            last_display_button_time = current_time;
        }
        
        last_display_button_state = current_display_button_state;
        
        sleep_ms(10);
    }
    
    return 0;
}
