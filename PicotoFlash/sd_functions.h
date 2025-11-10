/*
 * SD Card Functions Module Header
 */

#ifndef SD_FUNCTIONS_H
#define SD_FUNCTIONS_H

#include <stdbool.h>
#include "identification.h"
#include "ff.h"

// File definitions
#define CHIP_DATABASE_FILE "DATASHEET.csv"
#define BENCHMARK_LOG_FILE "benchmark_results_%04d%02d%02d.csv"
#define FORENSIC_REPORT_FILE "Report/forensic_report_%04d%02d%02d_%02d%02d%02d.txt"

// Constants
#define MAX_LINE_LENGTH 512
#define MAX_DATABASE_ENTRIES 100
#define MIN_SD_FREE_SPACE_MB 1

// Error codes
#define SUCCESS 0
#define ERROR_NO_DATABASE -1
#define ERROR_SD_NOT_PRESENT -2
#define ERROR_SD_FULL -3
#define ERROR_FILE_WRITE_FAIL -4
#define ERROR_FILE_NOT_FOUND -5
#define ERROR_DATABASE_CORRUPT -6
#define ERROR_SD_WRITE_FAIL -7
#define WARNING_EMPTY_DATABASE 1
#define WARNING_PARTIAL_DATABASE 2
#define WARNING_PERFORMANCE_OUTLIER 3
#define WARNING_INSUFFICIENT_DATA 4

// External global variables
extern FlashChipData database[];
extern int database_entry_count;
extern FlashChipData benchmark_results;
extern match_result_t match_results[];
extern FlashChipData test_chip;
extern bool database_loaded;

// Function declarations
void parse_csv_line(char* line, char fields[][MAX_FIELD_LENGTH], int* field_count);
bool validate_jedec_format(const char* jedec);
bool is_power_of_two(float capacity);
void get_timestamp(int* year, int* month, int* day, int* hour, int* min, int* sec);
bool check_sd_free_space(void);
int sd_load_chip_database(void);
int sd_log_benchmark_results(void);
int sd_create_forensic_report(void);

#endif // SD_FUNCTIONS_H
