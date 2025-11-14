/*
 * Complete Flash Chip Identification System
 * Master Pico Module
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
#include "ff.h"
#include "fatfs/FatFs_SPI/sd_driver/sd_card.h"
#include "identification.h"
#include "sd_functions.h"
#include "display_functions.h"

// Pin definitions
#define BUTTON_PIN 20
#define DISPLAY_BUTTON_PIN 21

// System constants
#define DEBOUNCE_DELAY_MS 50
#define MAX_MOUNT_ATTEMPTS 3
#define MOUNT_RETRY_DELAY_MS 500
#define POST_MOUNT_DELAY_MS 200

// Global variables
FlashChipData database[MAX_DATABASE_ENTRIES];
int database_entry_count = 0;
FlashChipData benchmark_results;
match_result_t match_results[TOP_MATCHES_COUNT];
bool database_loaded = false;

// Test data
FlashChipData test_chip = {
    .chip_model = "UNKNOWN",
    .company = "",
    .chip_family = "",
    .capacity_mbit = 128,
    .jedec_id = "EF 40 18",
    .read_speed_max = 6.25,
    .erase_speed = 150.0,
    .max_clock_freq_mhz = 50,
    .typ_4kb_erase_ms = 45.0,
    .max_4kb_erase_ms = 400.0,
    .typ_32kb_erase_ms = 120.0,
    .max_32kb_erase_ms = 1600.0,
    .typ_64kb_erase_ms = 150.0,
    .max_64kb_erase_ms = 2000.0,
    .typ_page_program_ms = 0.4,
    .max_page_program_ms = 3.0
};

// ============================================================================
// Main Function
// ============================================================================
int main(void) {
    stdio_init_all();
    
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
        
        // ===== GP20 BUTTON - RUN IDENTIFICATION =====
        if (last_button_state != current_button_state &&
            (current_time - last_button_time > DEBOUNCE_DELAY_MS) &&
            !current_button_state) {
            
            display_button_pressed_gp20();
            
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
                    printf("[ERROR] SD card not mounted after %d attempts\n", MAX_MOUNT_ATTEMPTS);
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
                sd_log_benchmark_results();
            }
            
            // Create forensic report
            sd_create_forensic_report();
            
            display_identification_complete();
            
            last_button_time = current_time;
        }
        
        last_button_state = current_button_state;
        
        // ===== GP21 BUTTON - VIEW DATABASE =====
        if (last_display_button_state != current_display_button_state &&
            (current_time - last_display_button_time > DEBOUNCE_DELAY_MS) &&
            !current_display_button_state) {
            
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
                    printf("[ERROR] SD card not mounted after %d attempts\n", MAX_MOUNT_ATTEMPTS);
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
