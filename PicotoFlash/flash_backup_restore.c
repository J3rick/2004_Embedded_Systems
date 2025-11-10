/*
 * flash_backup_restore.c
 * 
 * Robust backup/restore utilities for external SPI NOR flash chips.
 * 
 * Features:
 * - Page-aligned 256B writes with padding
 * - WREN (Write Enable) before every Page Program
 * - Global unprotect clears SR1+SR2 (BP bits, QE etc.)
 * - Progress indicators for user feedback
 * - Comprehensive error handling
 * - Handles destroyed SFDP tables (uses backup file size)
 * 
 * Dependencies: pico-sdk (spi/gpio), FatFs (ff.h)
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "ff.h"
#include "flash_backup_restore.h"

// ============================================================================
// SPI Configuration (match your hardware setup)
// ============================================================================
#define FLASH_SPI       spi0
#define PIN_CS          6

// ============================================================================
// Flash Command Opcodes
// ============================================================================
#define CMD_READ_SFDP   0x5A    // Read SFDP table
#define CMD_READ        0x03    // Read data
#define CMD_FAST_READ   0x0B    // Fast read (with dummy byte)
#define CMD_WREN        0x06    // Write enable
#define CMD_WRDI        0x04    // Write disable
#define CMD_RDSR1       0x05    // Read Status Register 1
#define CMD_RDSR2       0x35    // Read Status Register 2
#define CMD_WRSR        0x01    // Write Status Register
#define CMD_PP          0x02    // Page Program (256 bytes)
#define CMD_SE_4K       0x20    // 4KB sector erase
#define CMD_BE_64K      0xD8    // 64KB block erase
#define CMD_CE          0xC7    // Chip erase

// ============================================================================
// Low-level SPI Helper Functions
// ============================================================================

static inline void cs_low(void) {
    gpio_put(PIN_CS, 0);
}

static inline void cs_high(void) {
    gpio_put(PIN_CS, 1);
}

static inline void spi_tx(const uint8_t *buf, size_t len) {
    spi_write_blocking(FLASH_SPI, buf, len);
}

static inline void spi_rx(uint8_t *buf, size_t len) {
    spi_read_blocking(FLASH_SPI, 0x00, buf, len);
}

// ============================================================================
// Flash Status & Wait Functions
// ============================================================================

/**
 * Reads Flash Status Register 1 (SR1)
 */
static uint8_t flash_read_sr1(void) {
    uint8_t cmd = CMD_RDSR1;
    uint8_t sr = 0;
    
    cs_low();
    spi_tx(&cmd, 1);
    spi_rx(&sr, 1);
    cs_high();
    
    return sr;
}

/**
 * Waits for flash to complete current operation (WIP bit = 0)
 */
static void flash_wait_busy(void) {
    while (flash_read_sr1() & 0x01) {
        sleep_us(10);
    }
}

// ============================================================================
// Flash Protection Management
// ============================================================================

/**
 * Removes global write protection by clearing SR1 and SR2
 */
static void flash_global_unprotect(void) {
    uint8_t wren = CMD_WREN;
    uint8_t wrsr[3] = { CMD_WRSR, 0x00, 0x00 };  // Clear SR1 and SR2
    
    cs_low();
    spi_tx(&wren, 1);
    cs_high();
    
    cs_low();
    spi_tx(wrsr, 3);
    cs_high();
    
    flash_wait_busy();
}

// ============================================================================
// SFDP-based Size Detection with JEDEC ID Fallback
// ============================================================================

/**
 * Helper: Reads JEDEC ID (0x9F command)
 */
static void read_jedec_id_local(uint8_t out[3]) {
    uint8_t cmd = 0x9F;
    cs_low();
    spi_tx(&cmd, 1);
    spi_rx(out, 3);
    cs_high();
}

/**
 * Detects flash chip size using SFDP table with JEDEC ID fallback
 * 
 * Priority:
 *   1. SFDP table (most accurate)
 *   2. JEDEC ID capacity code (standard encoding)
 * 
 * JEDEC ID Format: [Manufacturer] [Type] [Capacity]
 *   Byte 3 (Capacity) = 0x14-0x1F encodes size as 2^N bytes
 *   Examples:
 *     0x13 = 2^19 =    512 KB
 *     0x14 = 2^20 =  1,024 KB = 1 MB
 *     0x15 = 2^21 =  2,048 KB = 2 MB
 *     0x16 = 2^22 =  4,096 KB = 4 MB
 *     0x17 = 2^23 =  8,192 KB = 8 MB
 *     0x18 = 2^24 = 16,384 KB = 16 MB
 *     0x19 = 2^25 = 32,768 KB = 32 MB
 *     0x1A = 2^26 = 65,536 KB = 64 MB
 *     0x1B = 2^27 = 131,072 KB = 128 MB
 *     0x1C = 2^28 = 262,144 KB = 256 MB
 * 
 * @return Flash size in bytes, or 0 if detection failed
 */
uint32_t flash_detect_size(void) {
    uint32_t size = 0;
    
    // ===================================================================
    // PRIORITY 1: Try SFDP table first (most accurate)
    // ===================================================================
    printf("[INFO] Attempting SFDP size detection...\n");
    
    // Read SFDP signature at address 0x000000
    uint8_t cmd[5] = { CMD_READ_SFDP, 0x00, 0x00, 0x00, 0x00 };  // +1 dummy byte
    uint8_t sfdp_sig[4];
    
    cs_low();
    spi_tx(cmd, 5);
    spi_rx(sfdp_sig, 4);
    cs_high();
    
    // Verify SFDP signature: "SFDP" (0x50444653)
    if (sfdp_sig[0] == 0x53 && sfdp_sig[1] == 0x46 && 
        sfdp_sig[2] == 0x44 && sfdp_sig[3] == 0x50) {
        
        printf("[DEBUG] SFDP signature found: %c%c%c%c\n",
               sfdp_sig[0], sfdp_sig[1], sfdp_sig[2], sfdp_sig[3]);
        
        // Read density field from SFDP Basic Parameter Table
        // Located at address 0x000004 (DWORD 2, bits 31:0)
        cmd[1] = 0x00;
        cmd[2] = 0x00;
        cmd[3] = 0x04;
        
        uint8_t density_bytes[4];
        cs_low();
        spi_tx(cmd, 5);
        spi_rx(density_bytes, 4);
        cs_high();
        
        // Reconstruct 32-bit density value (little-endian)
        uint32_t density_bits = (uint32_t)density_bytes[0] |
                               ((uint32_t)density_bytes[1] << 8) |
                               ((uint32_t)density_bytes[2] << 16) |
                               ((uint32_t)density_bytes[3] << 24);
        
        printf("[DEBUG] SFDP density field: 0x%08lX\n", density_bits);
        
        // Check if density uses N+1 encoding (bit 31 = 1)
        if (density_bits & 0x80000000) {
            // N+1 encoding: size = (N+1) bits
            uint32_t size_bits = (density_bits & 0x7FFFFFFF) + 1;
            size = (size_bits + 7) / 8;  // Convert bits to bytes
            printf("[SUCCESS] ✓ SFDP detection successful: %lu bytes (%.2f MB)\n",
                size, (float)size / (1024.0f * 1024.0f));
            return size;
        } else {
            // Direct encoding in bits (already in bits, not power-of-2)
            size = (density_bits + 1 + 7) / 8;  // ✅ CORRECT: convert bits to bytes
            printf("[SUCCESS] ✓ SFDP detection successful: %lu bytes (%.2f MB)\n",
                size, (float)size / (1024.0f * 1024.0f));
            return size;
        }

    }
    
    // ===================================================================
    // PRIORITY 2: Fall back to JEDEC ID capacity code
    // ===================================================================
    printf("[WARNING] SFDP signature not found, falling back to JEDEC ID...\n");
    
    uint8_t jedec[3];
    read_jedec_id_local(jedec);
    
    printf("[DEBUG] JEDEC ID: %02X %02X %02X\n", jedec[0], jedec[1], jedec[2]);
    printf("[DEBUG]   Manufacturer: 0x%02X\n", jedec[0]);
    printf("[DEBUG]   Device Type:  0x%02X\n", jedec[1]);
    printf("[DEBUG]   Capacity:     0x%02X\n", jedec[2]);
    
    // Extract capacity code (byte 3 of JEDEC ID)
    uint8_t capacity_code = jedec[2];
    
    // Standard capacity encoding: 2^N bytes
    // Most chips use 0x13 (512KB) to 0x1C (256MB)
    if (capacity_code >= 0x10 && capacity_code <= 0x1F) {
        size = (1UL << capacity_code);
        
        printf("[SUCCESS] ✓ JEDEC ID detection successful\n");
        printf("[INFO] Capacity code: 0x%02X = 2^%d = %lu bytes (%.2f MB)\n",
               capacity_code, capacity_code, size, 
               (float)size / (1024.0f * 1024.0f));
        
        return size;
    }
    
    // ===================================================================
    // Detection Failed
    // ===================================================================
    printf("[ERROR] Cannot determine chip size!\n");
    printf("[ERROR] SFDP: signature not found\n");
    printf("[ERROR] JEDEC ID: invalid capacity code (0x%02X)\n", capacity_code);
    printf("\n");
    printf("[INFO] Please check:\n");
    printf("[INFO]   1. SPI wiring (SCK, MOSI, MISO, CS)\n");
    printf("[INFO]   2. Flash chip power supply\n");
    printf("[INFO]   3. Chip compatibility with SFDP/JEDEC standards\n");
    printf("\n");
    
    return 0;  // Failed
}


// ============================================================================
// Backup Function
// ============================================================================

/**
 * Backs up entire flash contents to SD card file
 * 
 * @param filename  Path to backup file on SD card
 * @return true if successful, false otherwise
 */
bool flash_backup_to_sd(const char *filename) {
    printf("\n[BACKUP] Starting flash backup...\n");
    
    // Step 1: Detect chip size
    uint32_t size = flash_detect_size();
    if (size == 0) {
        printf("[ERROR] Flash chip size detection failed\n");
        return false;
    }
    printf("[INFO] Detected chip size: %lu bytes (%lu KB, %lu MB)\n",
           size, size / 1024, size / (1024 * 1024));
    
    // Step 2: Save original SPI speed and increase for faster backup
    uint32_t original_baudrate = spi_get_baudrate(FLASH_SPI);
    spi_set_baudrate(FLASH_SPI, 50 * 1000 * 1000);  // 50 MHz for fast read
    
    // Step 3: Create backup file on SD card
    FIL file;
    FRESULT fr = f_open(&file, filename, FA_WRITE | FA_CREATE_ALWAYS);
    if (fr != FR_OK) {
        printf("[ERROR] Failed to create backup file: %s (FatFS error: %d)\n",
               filename, fr);
        spi_set_baudrate(FLASH_SPI, original_baudrate);
        return false;
    }
    printf("[INFO] Backup file created: %s\n", filename);
    
    // Step 4: Allocate 64KB buffer for fast transfers
    uint8_t *buffer = malloc(65536);
    if (!buffer) {
        printf("[ERROR] Memory allocation failed (64KB buffer)\n");
        f_close(&file);
        spi_set_baudrate(FLASH_SPI, original_baudrate);
        return false;
    }
    
    // Step 5: Read flash and write to SD card
    uint32_t addr = 0;
    uint32_t total_chunks = (size + 65535) / 65536;
    uint32_t chunks_read = 0;
    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    
    printf("[INFO] Reading %lu chunks (65536 bytes each)...\n", total_chunks);
    
    while (addr < size) {
        // Calculate chunk size (last chunk may be smaller)
        uint32_t chunk_size = (size - addr > 65536) ? 65536 : (size - addr);
        
        // Fast read from flash (0x0B with dummy byte)
        uint8_t cmd[5] = { CMD_FAST_READ,
                           (uint8_t)(addr >> 16),
                           (uint8_t)(addr >> 8),
                           (uint8_t)addr,
                           0x00 };  // Dummy byte
        
        cs_low();
        spi_tx(cmd, 5);
        spi_rx(buffer, chunk_size);
        cs_high();
        
        // Write to SD card
        UINT bytes_written = 0;
        fr = f_write(&file, buffer, chunk_size, &bytes_written);
        if (fr != FR_OK || bytes_written != chunk_size) {
            printf("[ERROR] SD card write failed at offset %lu (FatFS error: %d)\n",
                   addr, fr);
            free(buffer);
            f_close(&file);
            spi_set_baudrate(FLASH_SPI, original_baudrate);
            return false;
        }
        
        addr += chunk_size;
        chunks_read++;
        
        // Progress indicator every 4 chunks (256KB) or at end
        if (chunks_read % 4 == 0 || addr >= size) {
            uint32_t elapsed_ms = to_ms_since_boot(get_absolute_time()) - start_time;
            float speed_mbps = (elapsed_ms > 0) ? 
                ((float)addr / 1024.0f / 1024.0f) / ((float)elapsed_ms / 1000.0f) : 0.0f;
            
            printf("[PROGRESS] Backed up %lu/%lu chunks (%.1f%%) - %.1f MB/s\n",
                   chunks_read, total_chunks,
                   (float)chunks_read * 100.0f / (float)total_chunks,
                   speed_mbps);
        }
    }
    
    free(buffer);
    f_close(&file);
    
    // Restore original SPI speed
    spi_set_baudrate(FLASH_SPI, original_baudrate);
    
    printf("[SUCCESS] Backup complete: %lu bytes written to %s\n", addr, filename);
    return true;
}

// ============================================================================
// Restore Function (with Known Size)
// ============================================================================

/**
 * Restores flash contents from SD card backup file
 * Uses known chip size instead of SFDP (which may be destroyed)
 * 
 * WARNING: This ERASES and REPROGRAMS the entire flash chip!
 * 
 * @param filename    Path to backup file on SD card
 * @param known_size  Flash chip size in bytes (from pre-backup detection)
 * @return true if successful, false otherwise
 */
bool flash_restore_from_sd_with_size(const char *filename, uint32_t known_size) {
    printf("\n[RESTORE] Starting flash restore...\n");
    printf("[WARNING] This will ERASE and REPROGRAM the entire flash chip!\n");
    printf("[INFO] Using pre-determined chip size: %lu bytes (%.2f MB)\n",
           known_size, (float)known_size / (1024.0f * 1024.0f));
    
    // Step 1: Open backup file
    FIL file;
    FRESULT fr = f_open(&file, filename, FA_READ);
    if (fr != FR_OK) {
        printf("[ERROR] Failed to open backup file: %s (FatFS error: %d)\n",
               filename, fr);
        return false;
    }
    
    DWORD file_size = f_size(&file);
    printf("[INFO] Backup file size: %lu bytes (%.2f MB)\n",
           file_size, (float)file_size / (1024.0f * 1024.0f));
    
    if (file_size > known_size) {
        printf("[WARNING] Backup size (%lu) exceeds chip size (%lu)\n",
               file_size, known_size);
        printf("[WARNING] Restore will be truncated to chip capacity\n");
    }
    
    // Use the smaller of the two sizes
    uint32_t restore_size = (file_size < known_size) ? file_size : known_size;
    
    // Step 2: Remove write protection
    printf("[INFO] Removing write protection...\n");
    flash_global_unprotect();
    printf("[INFO] Write protection cleared\n");
    
    // Step 3: Erase entire chip (4KB sectors)
    printf("[INFO] Erasing chip (%lu sectors @ 4KB each)...\n",
           (restore_size + 4095) / 4096);
    
    uint32_t sectors_erased = 0;
    uint32_t total_sectors = (restore_size + 4095) / 4096;
    
    for (uint32_t addr = 0; addr < restore_size; addr += 4096) {
        // Write enable
        uint8_t wren = CMD_WREN;
        cs_low();
        spi_tx(&wren, 1);
        cs_high();
        
        // Sector erase command
        uint8_t cmd[4] = { CMD_SE_4K,
                           (uint8_t)(addr >> 16),
                           (uint8_t)(addr >> 8),
                           (uint8_t)addr };
        cs_low();
        spi_tx(cmd, 4);
        cs_high();
        
        flash_wait_busy();
        sectors_erased++;
        
        // Progress indicator every 64 sectors (256KB) or at end
        if (sectors_erased % 64 == 0 || addr + 4096 >= restore_size) {
            printf("[PROGRESS] Erased %lu/%lu sectors (%.1f%%)\n",
                   sectors_erased, total_sectors,
                   (float)sectors_erased * 100.0f / (float)total_sectors);
        }
    }
    printf("[INFO] Erase complete: %lu sectors erased\n", sectors_erased);
    
    // Step 4: Program pages from backup file
    printf("[INFO] Programming flash from backup...\n");
    uint8_t page[256];
    uint32_t addr = 0;
    uint32_t pages_programmed = 0;
    uint32_t total_pages = (restore_size + 255) / 256;
    
    // Rewind file to start
    f_lseek(&file, 0);
    
    while (addr < restore_size) {
        // Read page from SD card (pad with 0xFF if partial)
        UINT bytes_read = 0;
        memset(page, 0xFF, sizeof(page));
        
        fr = f_read(&file, page, sizeof(page), &bytes_read);
        if (fr != FR_OK) {
            printf("[ERROR] SD card read failed at offset %lu (FatFS error: %d)\n",
                   addr, fr);
            f_close(&file);
            return false;
        }
        
        if (bytes_read == 0) {
            break;  // End of file
        }
        
        // Write enable
        uint8_t wren = CMD_WREN;
        cs_low();
        spi_tx(&wren, 1);
        cs_high();
        
        // Page program command + data
        uint8_t cmd[4] = { CMD_PP,
                           (uint8_t)(addr >> 16),
                           (uint8_t)(addr >> 8),
                           (uint8_t)addr };
        cs_low();
        spi_tx(cmd, 4);
        spi_tx(page, 256);
        cs_high();
        
        flash_wait_busy();
        
        addr += 256;
        pages_programmed++;
        
        // Progress indicator every 256 pages (64KB) or at end
        if (pages_programmed % 256 == 0 || addr >= restore_size) {
            printf("[PROGRESS] Programmed %lu/%lu pages (%.1f%%)\n",
                   pages_programmed, total_pages,
                   (float)pages_programmed * 100.0f / (float)total_pages);
        }
    }
    
    f_close(&file);
    printf("[SUCCESS] Restore complete: %lu pages programmed (%lu bytes)\n",
           pages_programmed, addr);
    
    return true;
}

/**
 * Legacy restore function (tries SFDP detection first)
 * Falls back to backup file size if SFDP fails
 */
bool flash_restore_from_sd(const char *filename) {
    printf("\n[RESTORE] Starting flash restore...\n");
    
    // Try SFDP detection first
    uint32_t chip_size = flash_detect_size();
    
    if (chip_size == 0) {
        printf("[WARNING] SFDP detection failed (chip may be erased)\n");
        printf("[INFO] Using backup file size instead...\n");
        
        // Open file to get size
        FIL file;
        FRESULT fr = f_open(&file, filename, FA_READ);
        if (fr != FR_OK) {
            printf("[ERROR] Failed to open backup file: %s\n", filename);
            return false;
        }
        
        chip_size = f_size(&file);
        f_close(&file);
        
        printf("[INFO] Using backup file size: %lu bytes (%.2f MB)\n",
               chip_size, (float)chip_size / (1024.0f * 1024.0f));
    }
    
    // Use size-aware restore
    return flash_restore_from_sd_with_size(filename, chip_size);
}

// ============================================================================
// Verify Function
// ============================================================================

/**
 * Verifies flash contents against SD card backup file
 * 
 * @param filename  Path to backup file on SD card
 * @return true if contents match, false if mismatch or error
 */
bool flash_verify_from_sd(const char *filename) {
    printf("\n[VERIFY] Starting verification...\n");
    
    // Open backup file
    FIL file;
    FRESULT fr = f_open(&file, filename, FA_READ);
    if (fr != FR_OK) {
        printf("[ERROR] Failed to open backup file: %s (FatFS error: %d)\n",
               filename, fr);
        return false;
    }
    
    DWORD file_size = f_size(&file);
    printf("[INFO] Verifying %lu bytes...\n", file_size);
    
    // Compare in 4KB chunks
    uint8_t flash_buf[4096];
    uint8_t file_buf[4096];
    uint32_t addr = 0;
    uint32_t total_chunks = (file_size + sizeof(flash_buf) - 1) / sizeof(flash_buf);
    uint32_t chunks_verified = 0;
    bool match = true;
    
    while (addr < file_size) {
        uint32_t chunk_size = (file_size - addr > sizeof(flash_buf)) ? 
                              sizeof(flash_buf) : (file_size - addr);
        
        // Read from flash
        uint8_t cmd[4] = { CMD_READ,
                           (uint8_t)(addr >> 16),
                           (uint8_t)(addr >> 8),
                           (uint8_t)addr };
        cs_low();
        spi_tx(cmd, 4);
        spi_rx(flash_buf, chunk_size);
        cs_high();
        
        // Read from file
        UINT bytes_read = 0;
        fr = f_read(&file, file_buf, chunk_size, &bytes_read);
        if (fr != FR_OK || bytes_read != chunk_size) {
            printf("[ERROR] SD card read failed at offset %lu\n", addr);
            f_close(&file);
            return false;
        }
        
        // Compare
        if (memcmp(flash_buf, file_buf, chunk_size) != 0) {
            printf("[ERROR] Verification failed at offset %lu!\n", addr);
            
            // Find first mismatch byte
            for (uint32_t i = 0; i < chunk_size; i++) {
                if (flash_buf[i] != file_buf[i]) {
                    printf("[ERROR] First mismatch at 0x%06lX: flash=0x%02X, file=0x%02X\n",
                           addr + i, flash_buf[i], file_buf[i]);
                    break;
                }
            }
            
            match = false;
            break;
        }
        
        addr += chunk_size;
        chunks_verified++;
        
        // Progress indicator every 64 chunks (256KB)
        if (chunks_verified % 64 == 0 || addr >= file_size) {
            printf("[PROGRESS] Verified %lu/%lu chunks (%.1f%%)\n",
                   chunks_verified, total_chunks,
                   (float)chunks_verified * 100.0f / (float)total_chunks);
        }
    }
    
    f_close(&file);
    
    if (match) {
        printf("[SUCCESS] Verification passed: Flash matches backup file perfectly!\n");
    }
    
    return match;
}
