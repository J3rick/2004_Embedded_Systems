/*
 * Display Functions Module Header
 * Contains output formatting functions for identification results and database display
 */

#ifndef DISPLAY_FUNCTIONS_H
#define DISPLAY_FUNCTIONS_H

#include <stdbool.h>
#include "identification.h"
#include "fatfs/FatFs_SPI/ff15/source/ff.h"

// External global variables
extern FlashChipData database[];
extern int database_entry_count;
extern match_result_t match_results[];
extern FlashChipData test_chip;
extern bool database_loaded;

// Function declarations
void display_system_banner(void);
void display_startup_instructions(void);
void display_sd_mount_attempt(int attempt, int max_attempts);
void display_sd_mount_success(void);
void display_sd_mount_failed(int max_attempts);
void display_sd_mount_warning(int error_code);
void display_database_loaded(int entry_count);
void display_sd_stabilization(void);
void display_button_pressed_gp20(void);
void display_button_pressed_gp21(void);
void display_database_reload_attempt(void);
void display_database_corrupt_warning(void);
void display_no_database_error(void);
void display_identification_complete(void);
void display_detailed_comparison(void);
void display_full_database(void);

#endif // DISPLAY_FUNCTIONS_H
