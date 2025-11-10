/*
 * flash_backup_restore.h
 * 
 * Flash Backup & Restore Utilities for External SPI NOR Flash
 * 
 * Features:
 * - SFDP-based automatic size detection
 * - Full chip backup to SD card
 * - Full chip restore from SD card (with/without SFDP)
 * - Data verification after restore
 * - Progress indicators for long operations
 * - Robust error handling
 */

#ifndef FLASH_BACKUP_RESTORE_H
#define FLASH_BACKUP_RESTORE_H

#include <stdint.h>
#include <stdbool.h>

/**
 * Detects flash chip size by reading SFDP table.
 * 
 * @return Flash size in bytes, or 0 if detection failed
 */
uint32_t flash_detect_size(void);

/**
 * Backs up entire flash chip contents to SD card file.
 * 
 * @param filename  Path to backup file on SD card
 * @return true if backup successful, false otherwise
 * 
 * @note Creates new file or overwrites existing file
 * @note Uses 64KB chunks and fast read (50 MHz) for speed
 */
bool flash_backup_to_sd(const char *filename);

/**
 * Restores flash chip contents from SD card backup file.
 * Auto-detects size from SFDP or uses backup file size.
 * 
 * WARNING: This will ERASE and REPROGRAM the entire flash chip!
 * 
 * @param filename  Path to backup file on SD card
 * @return true if restore successful, false otherwise
 * 
 * @note Automatically removes write protection
 * @note Erases entire chip before programming
 * @note Programs in 256-byte pages (page-aligned)
 */
bool flash_restore_from_sd(const char *filename);

/**
 * Restores flash chip contents with known size (SFDP not required).
 * Use this when benchmarks have destroyed the SFDP table.
 * 
 * WARNING: This will ERASE and REPROGRAM the entire flash chip!
 * 
 * @param filename    Path to backup file on SD card
 * @param known_size  Flash chip size in bytes
 * @return true if restore successful, false otherwise
 */
bool flash_restore_from_sd_with_size(const char *filename, uint32_t known_size);

/**
 * Verifies flash chip contents against SD card backup file.
 * 
 * @param filename  Path to backup file on SD card
 * @return true if contents match, false if mismatch or error
 * 
 * @note Compares in 4KB chunks
 * @note Reports first mismatch address if found
 */
bool flash_verify_from_sd(const char *filename);

#endif // FLASH_BACKUP_RESTORE_H
