/*
 * SD Card Functions Module
 * Contains SD card operations for database loading, logging, and reporting
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/rtc.h"
#include "ff.h"
#include "sd_functions.h"
#include "identification.h"
#include "read.h"
#include "erase.h"
#include "write.h"

// External references to global data
extern FlashChipData database[];
extern int database_entry_count;
extern FlashChipData benchmark_results;
extern match_result_t match_results[];
extern FlashChipData test_chip;

// ============================================================================
// Utility Functions
// ============================================================================

void parse_csv_line(char* line, char fields[][MAX_FIELD_LENGTH], int* field_count) {
    *field_count = 0;
    char* ptr = line;
    char* field_start = ptr;
    bool in_quotes = false;

    while (*ptr != '\0' && *ptr != '\n' && *ptr != '\r') {
        if (*ptr == '"') {
            in_quotes = !in_quotes;
        } else if (*ptr == ',' && !in_quotes) {
            int len = ptr - field_start;
            if (len >= MAX_FIELD_LENGTH) len = MAX_FIELD_LENGTH - 1;
            strncpy(fields[*field_count], field_start, len);
            fields[*field_count][len] = '\0';
            
            // Remove quotes
            if (fields[*field_count][0] == '"') {
                memmove(fields[*field_count], fields[*field_count] + 1, strlen(fields[*field_count]));
            }
            int last = strlen(fields[*field_count]) - 1;
            if (last >= 0 && fields[*field_count][last] == '"') {
                fields[*field_count][last] = '\0';
            }
            
            (*field_count)++;
            field_start = ptr + 1;
        }
        ptr++;
    }
    
    // Last field
    int len = ptr - field_start;
    if (len >= MAX_FIELD_LENGTH) len = MAX_FIELD_LENGTH - 1;
    strncpy(fields[*field_count], field_start, len);
    fields[*field_count][len] = '\0';
    
    if (fields[*field_count][0] == '"') {
        memmove(fields[*field_count], fields[*field_count] + 1, strlen(fields[*field_count]));
    }
    int last = strlen(fields[*field_count]) - 1;
    if (last >= 0 && fields[*field_count][last] == '"') {
        fields[*field_count][last] = '\0';
    }
    
    (*field_count)++;
}

bool validate_jedec_format(const char* jedec) {
    if (strlen(jedec) < 8) return false;
    int space_count = 0;
    for (int i = 0; jedec[i] != '\0'; i++) {
        if (jedec[i] == ' ') space_count++;
    }
    return (space_count == 2);
}

bool is_power_of_two(float capacity) {
    if (capacity <= 0) return false;
    int cap_int = (int)capacity;
    return (cap_int & (cap_int - 1)) == 0;
}

void get_timestamp(int* year, int* month, int* day, int* hour, int* min, int* sec) {
    datetime_t t;
    rtc_get_datetime(&t);
    *year = t.year;
    *month = t.month;
    *day = t.day;
    *hour = t.hour;
    *min = t.min;
    *sec = t.sec;
}

bool check_sd_free_space(void) {
    FATFS* fs;
    DWORD fre_clust, fre_sect;
    FRESULT res = f_getfree("0:", &fre_clust, &fs);
    
    if (res != FR_OK) {
        printf("[ERROR] ERROR_SD_NOT_PRESENT: Cannot access SD card\n");
        return false;
    }
    
    fre_sect = fre_clust * fs->csize;
    float free_mb = (float)(fre_sect) / 2048.0;
    
    printf("[INFO] SD Card Free Space: %.1f MB\n", free_mb);
    
    if (free_mb < MIN_SD_FREE_SPACE_MB) {
        printf("[ERROR] ERROR_SD_FULL: Less than %d MB free\n", MIN_SD_FREE_SPACE_MB);
        return false;
    }
    
    return true;
}

// ============================================================================
// D7.3.1: sd_load_chip_database()
// ============================================================================
int sd_load_chip_database(void) {
    FIL file;
    FRESULT fr;
    char line[MAX_LINE_LENGTH];
    
    printf("\n====================================\n");
    printf("  Loading Database from SD Card\n");
    printf("====================================\n");
    
    fr = f_open(&file, CHIP_DATABASE_FILE, FA_READ);
    if (fr != FR_OK) {
        printf("[ERROR] ERROR_FILE_NOT_FOUND: %s not found\n", CHIP_DATABASE_FILE);
        return ERROR_FILE_NOT_FOUND;
    }
    
    database_entry_count = 0;
    bool header_read = false;
    
    while (f_gets(line, sizeof(line), &file) != NULL) {
        if (!header_read) {
            header_read = true;
            continue;
        }
        
        if (database_entry_count >= MAX_DATABASE_ENTRIES) {
            printf("[WARNING] WARNING_PARTIAL_DATABASE: Max entries reached\n");
            break;
        }
        
        char fields[30][MAX_FIELD_LENGTH];
        int field_count;
        parse_csv_line(line, fields, &field_count);
        
        if (field_count < 15) continue;
        
        FlashChipData entry;
        memset(&entry, 0, sizeof(FlashChipData));
        
        strncpy(entry.chip_model, fields[0], MAX_FIELD_LENGTH - 1);
        strncpy(entry.company, fields[1], MAX_FIELD_LENGTH - 1);
        strncpy(entry.chip_family, fields[2], MAX_FIELD_LENGTH - 1);
        entry.capacity_mbit = atof(fields[3]);
        strncpy(entry.jedec_id, fields[4], MAX_FIELD_LENGTH - 1);
        
        // Validate
        if (!validate_jedec_format(entry.jedec_id)) continue;
        if (!is_power_of_two(entry.capacity_mbit)) continue;
        
        // Parse timing fields
        entry.typ_4kb_erase_ms = atof(fields[5]);
        entry.max_4kb_erase_ms = atof(fields[6]);
        entry.typ_32kb_erase_ms = atof(fields[7]);
        entry.max_32kb_erase_ms = atof(fields[8]);
        entry.typ_64kb_erase_ms = atof(fields[9]);
        entry.max_64kb_erase_ms = atof(fields[10]);
        entry.typ_page_program_ms = atof(fields[11]);
        entry.max_page_program_ms = atof(fields[12]);
        entry.max_clock_freq_mhz = atoi(fields[13]);
        entry.read_speed_max = atof(fields[14]);
        
        entry.erase_speed = entry.typ_64kb_erase_ms;
        
        database[database_entry_count++] = entry;
    }
    
    f_close(&file);
    
    if (database_entry_count == 0) {
        printf("[WARNING] WARNING_EMPTY_DATABASE: No valid entries\n");
        return WARNING_EMPTY_DATABASE;
    }
    
    printf("✓ Loaded %d chip entries from database\n", database_entry_count);
    return SUCCESS;
}

// ============================================================================
// D7.3.1: sd_log_benchmark_results()
// ============================================================================
int sd_log_benchmark_results(void) {
    if (!check_sd_free_space()) {
        printf("[ERROR] ERROR_SD_FULL: Insufficient free space\n");
        return ERROR_SD_FULL;
    }
    
    int year, month, day, hour, min, sec;
    get_timestamp(&year, &month, &day, &hour, &min, &sec);
    
    char filename[64];
    snprintf(filename, sizeof(filename), BENCHMARK_LOG_FILE, year, month, day);
    
    FIL file;
    FRESULT fr;
    bool file_exists = (f_stat(filename, NULL) == FR_OK);
    
    fr = f_open(&file, filename, FA_WRITE | FA_OPEN_APPEND);
    if (fr != FR_OK) {
        printf("[ERROR] ERROR_FILE_WRITE_FAIL: Cannot open log file\n");
        return ERROR_FILE_WRITE_FAIL;
    }
    
    // Declare ALL external benchmark data ONCE at the top
    extern int g_read_result_count;
    extern read_result_t g_read_results[];
    extern erase_result_t g_erase_result;
    extern int g_write_result_count;
    extern write_bench_capture_t g_write_results[];
    
    // ============================================================================
    // HEADER - Write if new file
    // ============================================================================
    if (!file_exists) {
        // Basic identification
        f_printf(&file, "Timestamp,JEDEC_ID,Manufacturer,PartNumber,Capacity_Mbit,");
        
        // Read benchmarks
        for (int i = 0; i < g_read_result_count && i < 8; i++) {
            if (!g_read_results[i].valid) continue;
            int clk = g_read_results[i].clock_mhz;
            f_printf(&file, "Read_%dMHz_1byte_MBps,", clk);
            f_printf(&file, "Read_%dMHz_page_MBps,", clk);
            f_printf(&file, "Read_%dMHz_sector_MBps,", clk);
            f_printf(&file, "Read_%dMHz_32KB_MBps,", clk);
            f_printf(&file, "Read_%dMHz_64KB_MBps,", clk);
        }
        f_printf(&file, "Read_50MHz_Derived_MBps,");
        
        // Write benchmarks - FIXED
        for (int i = 0; i < g_write_result_count && i < 8; i++) {
            if (!g_write_results[i].valid) continue;
            int clk = g_write_results[i].clock_mhz_actual;
            f_printf(&file, "Write_%dMHz_1byte_MBps,", clk);
            f_printf(&file, "Write_%dMHz_page_MBps,", clk);
            f_printf(&file, "Write_%dMHz_sector_MBps,", clk);
            f_printf(&file, "Write_%dMHz_32KB_MBps,", clk);
            f_printf(&file, "Write_%dMHz_64KB_MBps,", clk);
        }
        
        // Erase benchmarks
        f_printf(&file, "Erase_4KB_Avg_ms,Erase_4KB_Min_ms,Erase_4KB_Max_ms,");
        f_printf(&file, "Erase_32KB_Avg_ms,Erase_32KB_Min_ms,Erase_32KB_Max_ms,");
        f_printf(&file, "Erase_64KB_Avg_ms,Erase_64KB_Min_ms,Erase_64KB_Max_ms,");
        
        // Match information
        f_printf(&file, "Matched_Chip,Match_Confidence_Percent\n");
    }
    
    // ============================================================================
    // DATA ROW
    // ============================================================================
    
    // Timestamp
    f_printf(&file, "%04d-%02d-%02d %02d:%02d:%02d,", 
             year, month, day, hour, min, sec);
    
    // Basic chip identification
    f_printf(&file, "%s,", test_chip.jedec_id);
    f_printf(&file, "%s,", test_chip.company[0] != '\0' ? test_chip.company : "UNKNOWN");
    f_printf(&file, "%s,", test_chip.chip_model[0] != '\0' ? test_chip.chip_model : "UNKNOWN");
    f_printf(&file, "%.2f,", test_chip.capacity_mbit);
    
    // Read benchmarks
    for (int i = 0; i < g_read_result_count && i < 8; i++) {
        if (!g_read_results[i].valid) continue;
        for (int s = 0; s < 5; s++) {
            f_printf(&file, "%.4f,", g_read_results[i].size_stats[s].mb_s);
        }
    }
    f_printf(&file, "%.2f,", test_chip.read_speed_max);
    
    // Write benchmarks - FIXED
    for (int i = 0; i < g_write_result_count && i < 8; i++) {
        if (!g_write_results[i].valid) continue;
        for (int s = 0; s < g_write_results[i].num_results && s < 5; s++) {
            f_printf(&file, "%.4f,", g_write_results[i].results[s].stats.mb_s);
        }
    }
    
    // Erase benchmarks
    if (g_erase_result.valid) {
        f_printf(&file, "%.2f,%u,%u,", g_erase_result.avg_4k, g_erase_result.min_4k, g_erase_result.max_4k);
        f_printf(&file, "%.2f,%u,%u,", g_erase_result.avg_32k, g_erase_result.min_32k, g_erase_result.max_32k);
        f_printf(&file, "%.2f,%u,%u,", g_erase_result.avg_64k, g_erase_result.min_64k, g_erase_result.max_64k);
    } else {
        f_printf(&file, "0.00,0,0,0.00,0,0,0.00,0,0,");
    }
    
    // Match information
    if (match_results[0].database_index >= 0) {
        f_printf(&file, "%s %s,", 
                 match_results[0].chip_data.company,
                 match_results[0].chip_data.chip_model);
        f_printf(&file, "%.1f\n", match_results[0].confidence.overall_confidence);
    } else {
        f_printf(&file, "NO_MATCH,0.0\n");
    }
    
    f_close(&file);
    printf("✓ Comprehensive benchmark results logged to %s\n", filename);
    printf("  - %d read clock speeds\n", g_read_result_count);
    printf("  - %d write clock speeds\n", g_write_result_count);
    printf("  - Erase timings (4KB, 32KB, 64KB)\n");
    
    return SUCCESS;
}



// ============================================================================
// D7.3.2 & D7.3.3: sd_create_forensic_report()
// ============================================================================
int sd_create_forensic_report(void) {
    if (!check_sd_free_space()) {
        return ERROR_SD_FULL;
    }
    
    int year, month, day, hour, min, sec;
    get_timestamp(&year, &month, &day, &hour, &min, &sec);
    
    char filename[128];
    snprintf(filename, sizeof(filename), FORENSIC_REPORT_FILE, 
             year, month, day, hour, min, sec);
    
    // Create Report directory if it doesn't exist
    f_mkdir("Report");
    
    FIL file;
    FRESULT fr = f_open(&file, filename, FA_WRITE | FA_CREATE_ALWAYS);
    if (fr != FR_OK) {
        printf("[ERROR] ERROR_FILE_WRITE_FAIL: Cannot create report\n");
        return ERROR_FILE_WRITE_FAIL;
    }
    
    // Header
    f_printf(&file, "========================================\n");
    f_printf(&file, "  FLASH CHIP FORENSIC IDENTIFICATION REPORT\n");
    f_printf(&file, "========================================\n");
    f_printf(&file, "Generated: %04d-%02d-%02d %02d:%02d:%02d\n\n", 
             year, month, day, hour, min, sec);
    
    // Test Chip Data
    f_printf(&file, "--- Test Chip Benchmarks ---\n");
    f_printf(&file, "JEDEC ID: %s\n", test_chip.jedec_id);
    f_printf(&file, "Capacity: %.0f Mbit\n", test_chip.capacity_mbit);
    f_printf(&file, "Read Speed (50MHz): %.2f MB/s\n", test_chip.read_speed_max);
    f_printf(&file, "Erase Speed (64KB): %.1f ms\n", test_chip.erase_speed);
    f_printf(&file, "Max Clock: %d MHz\n\n", test_chip.max_clock_freq_mhz);
    
    // Identification Results
    f_printf(&file, "--- Identification Results ---\n");
    
    if (match_results[0].status == MATCH_FOUND) {
        f_printf(&file, "Status: FOUND (Exact Match)\n");
    } else if (match_results[0].status == MATCH_BEST_MATCH) {
        f_printf(&file, "Status: BEST MATCH\n");
    } else {
        f_printf(&file, "Status: UNKNOWN\n");
    }
    
    f_printf(&file, "Overall Confidence: %.1f%%\n\n", 
             match_results[0].confidence.overall_confidence);
    
    // Best Match Details
    if (match_results[0].database_index >= 0) {
        f_printf(&file, "--- Best Match Details ---\n");
        f_printf(&file, "Manufacturer: %s\n", match_results[0].chip_data.company);
        f_printf(&file, "Model: %s\n", match_results[0].chip_data.chip_model);
        f_printf(&file, "Family: %s\n", match_results[0].chip_data.chip_family);
        f_printf(&file, "JEDEC ID: %s\n", match_results[0].chip_data.jedec_id);
        f_printf(&file, "Capacity: %.0f Mbit\n\n", match_results[0].chip_data.capacity_mbit);
        
        // Confidence Breakdown
        f_printf(&file, "--- Confidence Factor Breakdown ---\n");
        if (match_results[0].confidence.breakdown.jedec_id_available) {
            f_printf(&file, "JEDEC ID Match (40%% weight): %.0f%%\n", 
                     match_results[0].confidence.breakdown.jedec_id_score);
        }
        if (match_results[0].confidence.breakdown.read_speed_available) {
            f_printf(&file, "Read Speed Match (20%% weight): %.0f%%\n", 
                     match_results[0].confidence.breakdown.read_speed_score);
        }
        if (match_results[0].confidence.breakdown.erase_speed_available) {
            f_printf(&file, "Erase Speed Match (10%% weight): %.0f%%\n", 
                     match_results[0].confidence.breakdown.erase_speed_score);
        }
        if (match_results[0].confidence.breakdown.clock_profile_available) {
            f_printf(&file, "Clock Profile Match (10%% weight): %.0f%%\n", 
                     match_results[0].confidence.breakdown.clock_profile_score);
        }
        f_printf(&file, "\n");
    }
    
    // Top 3 Matches
    f_printf(&file, "--- Top 3 Candidate Matches ---\n");
    for (int i = 0; i < TOP_MATCHES_COUNT; i++) {
        if (match_results[i].database_index >= 0) {
            f_printf(&file, "%d. %s %s (%.1f%% confidence)\n",
                     i + 1,
                     match_results[i].chip_data.company,
                     match_results[i].chip_data.chip_model,
                     match_results[i].confidence.overall_confidence);
        }
    }
    f_printf(&file, "\n");
    
    // Warnings
    if (match_results[0].has_outliers) {
        f_printf(&file, "--- Warnings ---\n");
        f_printf(&file, "WARNING_PERFORMANCE_OUTLIER: Significant performance deviations detected\n");
    }
    if (strlen(match_results[0].confidence.warning_message) > 0) {
        f_printf(&file, "%s\n", match_results[0].confidence.warning_message);
    }
    
    f_printf(&file, "\n========================================\n");
    f_printf(&file, "End of Report\n");
    f_printf(&file, "========================================\n");
    
    f_close(&file);
    
    printf("✓ Forensic report saved: %s\n", filename);
    return SUCCESS;
}
