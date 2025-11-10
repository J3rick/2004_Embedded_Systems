/*
 * Flash Chip Identification Module Header
 * Contains data structures and function declarations for chip identification
 */

#ifndef IDENTIFICATION_H
#define IDENTIFICATION_H

#include <stdbool.h>

// Constants
#define MAX_FIELD_LENGTH 64
#define TOP_MATCHES_COUNT 3

// Match status enumeration
typedef enum {
    MATCH_UNKNOWN,
    MATCH_BEST_MATCH,
    MATCH_FOUND
} match_status_t;

// Benchmark data structure
typedef struct {
    char chip_model[MAX_FIELD_LENGTH];
    char company[MAX_FIELD_LENGTH];
    char chip_family[MAX_FIELD_LENGTH];
    float capacity_mbit;
    char jedec_id[MAX_FIELD_LENGTH];
    
    // Performance benchmarks
    float read_speed_max;       // From CSV field 14 (50mhz_read_speed)
    float erase_speed;          // From CSV fields 9-10 (typ_64kb_erase)
    int max_clock_freq_mhz;
    
    // Detailed timing data
    float typ_4kb_erase_ms;
    float max_4kb_erase_ms;
    float typ_32kb_erase_ms;
    float max_32kb_erase_ms;
    float typ_64kb_erase_ms;
    float max_64kb_erase_ms;
    float typ_page_program_ms;
    float max_page_program_ms;
} FlashChipData;

// Factor confidence breakdown
typedef struct {
    float jedec_id_score;       // 0-100
    float read_speed_score;     // 0-100
    float write_speed_score;    // 0-100 (always 0 - not in CSV)
    float erase_speed_score;    // 0-100
    float clock_profile_score;  // 0-100
    
    bool jedec_id_available;
    bool read_speed_available;
    bool write_speed_available; // Always false
    bool erase_speed_available;
    bool clock_profile_available;
} factor_breakdown_t;

// Confidence result structure
typedef struct {
    float overall_confidence;   // 0-100%
    factor_breakdown_t breakdown;
    int factors_used;
    char warning_message[256];
} confidence_result_t;

// Match result structure
typedef struct {
    FlashChipData chip_data;
    confidence_result_t confidence;
    match_status_t status;
    int database_index;
    bool has_outliers;
} match_result_t;

// External global variables
extern FlashChipData database[];
extern int database_entry_count;
extern match_result_t match_results[];

// Function declarations
confidence_result_t chip_calculate_confidence(FlashChipData* measured, FlashChipData* expected);
match_status_t chip_match_database(FlashChipData* test_data);

#endif // IDENTIFICATION_H
