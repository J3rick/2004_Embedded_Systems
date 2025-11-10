/*
 * Display Functions Module
 * Contains all output formatting functions for identification and database display
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "display_functions.h"
#include "identification.h"
#include"read.h"
#include"write.h"
#include"erase.h"

// External references
extern FlashChipData database[];
extern int database_entry_count;
extern match_result_t match_results[];
extern FlashChipData test_chip;
extern bool database_loaded;

// ============================================================================
// System startup displays
// ============================================================================

void display_system_banner(void) {
    printf("\n\n");
    printf("========================================\n");
    printf("  Flash Chip Identification System\n");
    printf("  Master Pico Module\n");
    printf("========================================\n\n");
}

// void display_startup_instructions(void) {
//     printf("Press GP20 to run identification\n");
//     printf("Press GP21 to view database\n\n");
// }

void display_sd_mount_attempt(int attempt, int max_attempts) {
    printf("Mounting SD card (attempt %d/%d)...\n", attempt, max_attempts);
}

void display_sd_mount_success(void) {
    printf("✓ SD card mounted successfully\n");
}

void display_sd_stabilization(void) {
    printf("Waiting for SD card stabilization...\n");
    printf("✓ SD card ready\n\n");
}

void display_sd_mount_warning(int error_code) {
    printf("[WARN] Mount failed with error %d, retrying...\n", error_code);
}

void display_sd_mount_failed(int max_attempts) {
    printf("[ERROR] ERROR_SD_NOT_PRESENT: SD card mount failed after %d attempts\n", max_attempts);
    printf("[INFO] System will continue without SD card\n");
    printf("[INFO] Press GP20 to retry SD card mounting\n\n");
}

void display_database_loaded(int entry_count) {
    printf("✓ Database loaded: %d entries\n\n", entry_count);
}

// ============================================================================
// Button press displays
// ============================================================================

void display_button_pressed_gp20(void) {
    printf("\n[Button GP20 pressed]\n");
}

void display_button_pressed_gp21(void) {
    printf("\n[Button GP21 pressed - Database View]\n");
}

void display_database_reload_attempt(void) {
    printf("Attempting to reload database...\n");
}

void display_database_corrupt_warning(void) {
    printf("[WARN] Database read errors detected. Remounting SD card...\n");
}

void display_no_database_error(void) {
    printf("[ERROR] ERROR_NO_DATABASE: Database not loaded\n");
    printf("[INFO] Cannot perform identification without database\n");
}

void display_identification_complete(void) {
    printf("\n[INFO] Identification complete. Press button again for next chip.\n\n");
}

// ============================================================================
// Display consolidated chip information before database matching
// ============================================================================
void display_consolidated_chip_info(void) {  // ← REMOVE "static"
    // Declare external variables from picotoflash.c
    extern FlashChipData test_chip;
    extern int g_read_result_count;
    extern read_result_t g_read_results[];
    extern int g_write_result_count;
    extern write_bench_capture_t g_write_results[];
    extern erase_result_t g_erase_result;
    
    printf("\n");
    printf("*******************************************************\n");
    printf(" CONSOLIDATED CHIP INFORMATION\n");
    printf("*******************************************************\n");
    
    // Identification Data
    printf("\n--- CHIP IDENTIFICATION ---\n");
    printf("  JEDEC ID        : %s\n", test_chip.jedec_id);
    printf("  Capacity        : %.2f Mbit (%.2f MB)\n", 
           test_chip.capacity_mbit, test_chip.capacity_mbit / 8.0f);
    printf("  Manufacturer    : %s\n", 
           test_chip.company[0] != '\0' ? test_chip.company : "Unknown");
    printf("  Part Number     : %s\n", 
           test_chip.chip_model[0] != '\0' ? test_chip.chip_model : "Unknown");
    
    // Read Performance
    printf("\n--- READ PERFORMANCE ---\n");
    if (g_read_result_count > 0) {
        for (int i = 0; i < g_read_result_count && i < 8; i++) {
            if (!g_read_results[i].valid) continue;
            printf("  @ %d MHz:\n", g_read_results[i].clock_mhz);
            printf("    - 1-byte     : %.2f MB/s\n", g_read_results[i].size_stats[0].mb_s);
            printf("    - Page (256B): %.2f MB/s\n", g_read_results[i].size_stats[1].mb_s);
            printf("    - Sector (4K): %.2f MB/s\n", g_read_results[i].size_stats[2].mb_s);
            printf("    - Block (32K): %.2f MB/s\n", g_read_results[i].size_stats[3].mb_s);
            printf("    - Block (64K): %.2f MB/s\n", g_read_results[i].size_stats[4].mb_s);
        }
        printf("  Derived @ 50 MHz: %.2f MB/s\n", test_chip.read_speed_max);
    } else {
        printf("  No read benchmarks available\n");
    }
    
    // Write Performance
    printf("\n--- WRITE PERFORMANCE ---\n");
    if (g_write_result_count > 0) {
        for (int i = 0; i < g_write_result_count && i < 8; i++) {
            if (!g_write_results[i].valid) continue;
            printf("  @ %d MHz (actual: %d MHz):\n", 
                   g_write_results[i].clock_mhz_requested,
                   g_write_results[i].clock_mhz_actual);
            
            for (int s = 0; s < g_write_results[i].num_results && s < 5; s++) {
                printf("    - %-12s: %.2f MB/s (avg: %.1f µs)\n",
                       g_write_results[i].results[s].label,
                       g_write_results[i].results[s].stats.mb_s,
                       g_write_results[i].results[s].stats.avg_us);
            }
        }
    } else {
        printf("  Write benchmarks disabled or not available\n");
    }
    
    // Erase Performance
    printf("\n--- ERASE PERFORMANCE ---\n");
    if (g_erase_result.valid) {
        printf("  4KB Sector:\n");
        printf("    - Average     : %.2f ms\n", g_erase_result.avg_4k);
        printf("    - Min / Max   : %u ms / %u ms\n", 
               g_erase_result.min_4k, g_erase_result.max_4k);
        
        printf("  32KB Block:\n");
        printf("    - Average     : %.2f ms\n", g_erase_result.avg_32k);
        printf("    - Min / Max   : %u ms / %u ms\n", 
               g_erase_result.min_32k, g_erase_result.max_32k);
        
        printf("  64KB Block:\n");
        printf("    - Average     : %.2f ms\n", g_erase_result.avg_64k);
        printf("    - Min / Max   : %u ms / %u ms\n", 
               g_erase_result.min_64k, g_erase_result.max_64k);
    } else {
        printf("  No erase benchmarks available\n");
    }
    
    printf("\n*******************************************************\n");
    printf(" Ready to match against database...\n");
    printf("*******************************************************\n\n");
}


// ============================================================================
// Display detailed comparison of test chip with top 3 matches
// ============================================================================
void display_detailed_comparison(void) {
    // Display outlier warnings
    if (match_results[0].has_outliers) {
        printf("  Performance outliers detected!\n");
    }
    
    printf("\n");
    
    // Display detailed comparison with top 3 matches
    printf("--- TOP 3 MATCHES WITH FACTOR BREAKDOWN ---\n\n");
    
    for (int i = 0; i < TOP_MATCHES_COUNT; i++) {
        if (match_results[i].database_index >= 0) {
            printf("RANK %d: %s %s\n",
                   i + 1,
                   match_results[i].chip_data.company,
                   match_results[i].chip_data.chip_model);
            printf("  Overall Confidence: %.1f%%\n", match_results[i].confidence.overall_confidence);
            printf("\n  DATABASE VALUES:\n");
            printf("    JEDEC ID:          %s\n", match_results[i].chip_data.jedec_id);
            printf("    Read Speed:        %.2f MB/s\n", match_results[i].chip_data.read_speed_max);
            printf("    Erase Speed:       %.2f ms (typ 64KB)\n", match_results[i].chip_data.erase_speed);
            printf("    Max Clock Freq:    %d MHz\n", match_results[i].chip_data.max_clock_freq_mhz);
            printf("    Page Program:      %.2f ms (typ)\n", match_results[i].chip_data.typ_page_program_ms);
            printf("    Capacity:          %.1f Mbit\n", match_results[i].chip_data.capacity_mbit);
            
            printf("\n  MATCHING FACTORS:\n");
            
            // JEDEC ID
            printf("    [%.1f%%] JEDEC ID: ", match_results[i].confidence.breakdown.jedec_id_score);
            if (match_results[i].confidence.breakdown.jedec_id_available) {
                if (strcmp(test_chip.jedec_id, match_results[i].chip_data.jedec_id) == 0) {
                    printf("✓ MATCH (%s = %s)\n",
                           test_chip.jedec_id,
                           match_results[i].chip_data.jedec_id);
                } else {
                    printf("✗ MISMATCH (%s ≠ %s)\n",
                           test_chip.jedec_id,
                           match_results[i].chip_data.jedec_id);
                }
            } else {
                printf("N/A (missing data)\n");
            }
            
            // Read Speed
            printf("    [%.1f%%] READ SPEED: ", match_results[i].confidence.breakdown.read_speed_score);
            if (match_results[i].confidence.breakdown.read_speed_available) {
                float read_diff = test_chip.read_speed_max - match_results[i].chip_data.read_speed_max;
                float read_pct = (read_diff / match_results[i].chip_data.read_speed_max) * 100.0;
                if (fabs(read_pct) < 15.0) {
                    printf("✓ CLOSE (test: %.2f, db: %.2f, diff: %+.1f%%)\n",
                           test_chip.read_speed_max,
                           match_results[i].chip_data.read_speed_max,
                           read_pct);
                } else {
                    printf("✗ DIFFERS (test: %.2f, db: %.2f, diff: %+.1f%%)\n",
                           test_chip.read_speed_max,
                           match_results[i].chip_data.read_speed_max,
                           read_pct);
                }
            } else {
                printf("N/A (missing data)\n");
            }
            
            // Erase Speed
            printf("    [%.1f%%] ERASE SPEED: ", match_results[i].confidence.breakdown.erase_speed_score);
            if (match_results[i].confidence.breakdown.erase_speed_available) {
                float erase_diff = test_chip.erase_speed - match_results[i].chip_data.erase_speed;
                float erase_pct = (erase_diff / match_results[i].chip_data.erase_speed) * 100.0;
                if (fabs(erase_pct) < 20.0) {
                    printf("✓ CLOSE (test: %.2f, db: %.2f, diff: %+.1f%%)\n",
                           test_chip.erase_speed,
                           match_results[i].chip_data.erase_speed,
                           erase_pct);
                } else {
                    printf("✗ DIFFERS (test: %.2f, db: %.2f, diff: %+.1f%%)\n",
                           test_chip.erase_speed,
                           match_results[i].chip_data.erase_speed,
                           erase_pct);
                }
            } else {
                printf("N/A (missing data)\n");
            }
            
            // Clock Profile
            printf("    [%.1f%%] CLOCK PROFILE: ", match_results[i].confidence.breakdown.clock_profile_score);
            if (match_results[i].confidence.breakdown.clock_profile_available) {
                float clock_diff = test_chip.max_clock_freq_mhz - match_results[i].chip_data.max_clock_freq_mhz;
                float clock_pct = (clock_diff / match_results[i].chip_data.max_clock_freq_mhz) * 100.0;
                if (fabs(clock_pct) < 15.0) {
                    printf("✓ MATCH (test: %d, db: %d, diff: %+.1f%%)\n",
                           test_chip.max_clock_freq_mhz,
                           match_results[i].chip_data.max_clock_freq_mhz,
                           clock_pct);
                } else {
                    printf("✗ DIFFERS (test: %d, db: %d, diff: %+.1f%%)\n",
                           test_chip.max_clock_freq_mhz,
                           match_results[i].chip_data.max_clock_freq_mhz,
                           clock_pct);
                }
            } else {
                printf("N/A (missing data)\n");
            }
            
            printf("\n");
        }
    }
    
    printf("====================================\n\n");
}

// ============================================================================
// Display full database contents in formatted table
// ============================================================================
void display_full_database(void) {
    printf("\n========================================\n");
    printf("        DATABASE CONTENTS\n");
    printf("========================================\n\n");
    
    if (database_entry_count == 0) {
        printf("Database is empty or not loaded.\n");
        if (!database_loaded) {
            printf("Database file may be missing from SD card.\n");
        }
    } else {
        printf("Total entries: %d\n\n", database_entry_count);
        
        // Print header row
        printf("%-4s %-20s %-30s %-15s %-12s %-10s %-8s %-8s\n",
               "No.", "Company", "Chip Model", "Family", "JEDEC ID",
               "Cap(Mb)", "MaxClk", "Read");
        printf("%-4s %-20s %-30s %-15s %-12s %-10s %-8s %-8s\n",
               "----", "--------------------", "------------------------------",
               "---------------", "------------", "----------", "--------", "--------");
        
        // Print all database entries
        for (int i = 0; i < database_entry_count; i++) {
            printf("%-4d %-20s %-30s %-15s %-12s %-10.1f %-8d %-8.2f\n",
                   i + 1,
                   database[i].company,
                   database[i].chip_model,
                   database[i].chip_family,
                   database[i].jedec_id,
                   database[i].capacity_mbit,
                   database[i].max_clock_freq_mhz,
                   database[i].read_speed_max);
        }
        
        printf("\n");
        printf("Performance details available:\n");
        printf("- 4KB/32KB/64KB erase times (typ/max)\n");
        printf("- Page program times (typ/max)\n");
        printf("- Read/Erase speeds for matching\n");
    }
    
    printf("========================================\n\n");
}
