/*
 * flash_backup_restore.c
 * 
 * Enhanced backup/restore utilities for external SPI NOR flash chips
 * 
 * Features:
 * - Supports chips up to 4GB (with 4-byte addressing)
 * - Page-aligned 256B writes with padding
 * - WREN (Write Enable) before every Page Program
 * - Global unprotect clears SR1+SR2 (BP bits, QE etc.)
 * - Progress indicators for user feedback
 * - Comprehensive error handling
 * - Smart chip erase (fastest method)
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
#define FLASH_SPI spi0
#define PIN_CS 6

// ============================================================================
// Flash Command Opcodes (3-byte addressing)
// ============================================================================
#define CMD_READ_SFDP  0x5A  // Read SFDP table
#define CMD_READ       0x03  // Read data
#define CMD_FAST_READ  0x0B  // Fast read (with dummy byte)
#define CMD_WREN       0x06  // Write enable
#define CMD_WRDI       0x04  // Write disable
#define CMD_RDSR1      0x05  // Read Status Register 1
#define CMD_RDSR2      0x35  // Read Status Register 2
#define CMD_WRSR       0x01  // Write Status Register
#define CMD_PP         0x02  // Page Program (256 bytes)
#define CMD_SE_4K      0x20  // 4KB sector erase
#define CMD_BE_64K     0xD8  // 64KB block erase
#define CMD_CE         0xC7  // Chip erase

// ============================================================================
// Flash Command Opcodes (4-byte addressing for chips >16MB)
// ============================================================================
#define CMD_FAST_READ_4B  0x0C  // Fast read (4-byte address)
#define CMD_PP_4B         0x12  // Page Program (4-byte address)
#define CMD_SE_4K_4B      0x21  // 4KB sector erase (4-byte address)
#define CMD_BE_64K_4B     0xDC  // 64KB block erase (4-byte address)
#define CMD_EN4B          0xB7  // Enter 4-byte address mode
#define CMD_EX4B          0xE9  // Exit 4-byte address mode

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
// 4-Byte Addressing Mode Control
// ============================================================================
/**
 * Enters 4-byte addressing mode (for chips >16 MB)
 */
static void flash_enter_4byte_mode(void) {
    uint8_t cmd = CMD_EN4B;
    cs_low();
    spi_tx(&cmd, 1);
    cs_high();
    printf("[INFO] Enabled 4-byte addressing mode\n");
}

/**
 * Exits 4-byte addressing mode (return to 3-byte)
 */
static void flash_exit_4byte_mode(void) {
    uint8_t cmd = CMD_EX4B;
    cs_low();
    spi_tx(&cmd, 1);
    cs_high();
    printf("[INFO] Disabled 4-byte addressing mode\n");
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
 * @return Flash size in bytes, or 0 if detection failed
 */
uint32_t flash_detect_size(void) {
    uint32_t size = 0;

    // Try SFDP table first
    printf("[INFO] Attempting SFDP size detection...\n");
    uint8_t cmd[5] = { CMD_READ_SFDP, 0x00, 0x00, 0x00, 0x00 };
    uint8_t sfdp_sig[4];
    cs_low();
    spi_tx(cmd, 5);
    spi_rx(sfdp_sig, 4);
    cs_high();

    // Verify SFDP signature: "SFDP"
    if (sfdp_sig[0] == 0x53 && sfdp_sig[1] == 0x46 &&
        sfdp_sig[2] == 0x44 && sfdp_sig[3] == 0x50) {
        
        printf("[DEBUG] SFDP signature found\n");
        
        // Read density field
        cmd[3] = 0x04;
        uint8_t density_bytes[4];
        cs_low();
        spi_tx(cmd, 5);
        spi_rx(density_bytes, 4);
        cs_high();

        uint32_t density_bits = (uint32_t)density_bytes[0] |
                               ((uint32_t)density_bytes[1] << 8) |
                               ((uint32_t)density_bytes[2] << 16) |
                               ((uint32_t)density_bytes[3] << 24);

        if (density_bits & 0x80000000) {
            uint32_t size_bits = (density_bits & 0x7FFFFFFF) + 1;
            size = (size_bits + 7) / 8;
        } else {
            size = (density_bits + 7) / 8;
        }
        
        printf("[SUCCESS] SFDP detection: %lu bytes (%.2f MB)\n",
               size, (float)size / (1024.0f * 1024.0f));
        return size;
    }

    // Fall back to JEDEC ID
    printf("[WARNING] SFDP not found, using JEDEC ID...\n");
    uint8_t jedec[3];
    read_jedec_id_local(jedec);
    printf("[DEBUG] JEDEC ID: %02X %02X %02X\n", jedec[0], jedec[1], jedec[2]);

    uint8_t capacity_code = jedec[2];
    if (capacity_code >= 0x10 && capacity_code <= 0x1F) {
        size = (1UL << capacity_code);
        printf("[SUCCESS] JEDEC ID detection: %lu bytes (%.2f MB)\n",
               size, (float)size / (1024.0f * 1024.0f));
        return size;
    }

    printf("[ERROR] Cannot determine chip size!\n");
    return 0;
}

// ============================================================================
// Backup Function (with 4-byte addressing support)
// ============================================================================
/**
 * Backs up entire flash contents to SD card file
 * Supports chips up to 4GB with automatic 4-byte addressing
 */
bool flash_backup_to_sd(const char *filename) {
    printf("\n[BACKUP] Starting flash backup...\n");

    // Step 1: Detect chip size
    uint32_t size = flash_detect_size();
    if (size == 0) {
        printf("[ERROR] Flash chip size detection failed\n");
        return false;
    }

    printf("[INFO] Detected chip size: %lu bytes (%lu MB)\n",
           size, size / (1024 * 1024));

    // Determine if 4-byte addressing is needed
    bool use_4byte = (size > 16 * 1024 * 1024);  // >16MB needs 4-byte
    if (use_4byte) {
        printf("[INFO] Chip >16MB: Enabling 4-byte addressing mode\n");
        flash_enter_4byte_mode();
    }

    // Step 2: Set SPI speed
    uint32_t original_baudrate = spi_get_baudrate(FLASH_SPI);
    spi_set_baudrate(FLASH_SPI, 50 * 1000 * 1000);  // 50 MHz

    // Step 3: Create backup file
    FIL file;
    FRESULT fr = f_open(&file, filename, FA_WRITE | FA_CREATE_ALWAYS);
    if (fr != FR_OK) {
        printf("[ERROR] Failed to create backup file (error: %d)\n", fr);
        if (use_4byte) flash_exit_4byte_mode();
        spi_set_baudrate(FLASH_SPI, original_baudrate);
        return false;
    }

    printf("[INFO] Backup file created: %s\n", filename);

    // Step 4: Allocate buffer
    uint8_t *buffer = malloc(65536);
    if (!buffer) {
        printf("[ERROR] Memory allocation failed\n");
        f_close(&file);
        if (use_4byte) flash_exit_4byte_mode();
        spi_set_baudrate(FLASH_SPI, original_baudrate);
        return false;
    }

    // Step 5: Read flash and write to SD
    uint32_t addr = 0;
    uint32_t total_chunks = (size + 65535) / 65536;
    uint32_t chunks_read = 0;
    uint32_t start_time = to_ms_since_boot(get_absolute_time());

    printf("[INFO] Reading %lu chunks (65536 bytes each)...\n", total_chunks);

    while (addr < size) {
        uint32_t chunk_size = (size - addr > 65536) ? 65536 : (size - addr);

        // Fast read with appropriate addressing
        if (use_4byte) {
            // 4-byte Fast Read (0x0C)
            uint8_t cmd[6] = { CMD_FAST_READ_4B,
                              (uint8_t)(addr >> 24),
                              (uint8_t)(addr >> 16),
                              (uint8_t)(addr >> 8),
                              (uint8_t)addr,
                              0x00 };  // Dummy byte
            cs_low();
            spi_tx(cmd, 6);
            spi_rx(buffer, chunk_size);
            cs_high();
        } else {
            // 3-byte Fast Read (0x0B)
            uint8_t cmd[5] = { CMD_FAST_READ,
                              (uint8_t)(addr >> 16),
                              (uint8_t)(addr >> 8),
                              (uint8_t)addr,
                              0x00 };  // Dummy byte
            cs_low();
            spi_tx(cmd, 5);
            spi_rx(buffer, chunk_size);
            cs_high();
        }

        // Write to SD card
        UINT bytes_written = 0;
        fr = f_write(&file, buffer, chunk_size, &bytes_written);
        if (fr != FR_OK || bytes_written != chunk_size) {
            printf("[ERROR] SD write failed at offset %lu\n", addr);
            free(buffer);
            f_close(&file);
            if (use_4byte) flash_exit_4byte_mode();
            spi_set_baudrate(FLASH_SPI, original_baudrate);
            return false;
        }

        addr += chunk_size;
        chunks_read++;

        // Progress indicator
        if (chunks_read % 4 == 0 || addr >= size) {
            uint32_t elapsed_ms = to_ms_since_boot(get_absolute_time()) - start_time;
            float speed_mbps = (elapsed_ms > 0) ?
                ((float)addr / 1024.0f / 1024.0f) / ((float)elapsed_ms / 1000.0f) : 0.0f;
            printf("[PROGRESS] %lu/%lu chunks (%.1f%%) - %.1f MB/s\n",
                   chunks_read, total_chunks,
                   (float)chunks_read * 100.0f / (float)total_chunks,
                   speed_mbps);
        }
    }

    // Cleanup
    free(buffer);
    f_close(&file);
    if (use_4byte) flash_exit_4byte_mode();
    spi_set_baudrate(FLASH_SPI, original_baudrate);

    printf("[SUCCESS] Backup complete: %lu bytes → %s\n", addr, filename);
    return true;
}

// ============================================================================
// Smart Erase Function (with 4-byte addressing support)
// ============================================================================
/**
 * Smart erase: Uses chip erase for full restore, block erase for partial
 */
static bool flash_smart_erase(uint32_t restore_size, uint32_t chip_size, bool use_4byte) {
    // Strategy 1: Chip Erase (fastest for full chip)
    if (restore_size == chip_size) {
        printf("[INFO] Using Chip Erase (fastest method)...\n");
        
        uint8_t wren = CMD_WREN;
        cs_low();
        spi_tx(&wren, 1);
        cs_high();

        uint8_t cmd = CMD_CE;  // 0xC7
        cs_low();
        spi_tx(&cmd, 1);
        cs_high();

        printf("[INFO] Chip erase in progress (30-120s)...\n");
        uint32_t start = to_ms_since_boot(get_absolute_time());
        flash_wait_busy();
        uint32_t elapsed = to_ms_since_boot(get_absolute_time()) - start;
        printf("[INFO] Chip erase complete: %.1f seconds\n",
               (float)elapsed / 1000.0f);
        return true;
    }

    // Strategy 2: 64KB Block Erase
    printf("[INFO] Using 64KB block erase...\n");
    uint32_t addr = 0;
    uint32_t blocks_erased = 0;

    while (addr < restore_size) {
        if ((restore_size - addr >= 65536) && (addr % 65536 == 0)) {
            // 64KB block erase
            uint8_t wren = CMD_WREN;
            cs_low();
            spi_tx(&wren, 1);
            cs_high();

            if (use_4byte) {
                uint8_t cmd[5] = { CMD_BE_64K_4B,
                                  (uint8_t)(addr >> 24),
                                  (uint8_t)(addr >> 16),
                                  (uint8_t)(addr >> 8),
                                  (uint8_t)addr };
                cs_low();
                spi_tx(cmd, 5);
                cs_high();
            } else {
                uint8_t cmd[4] = { CMD_BE_64K,
                                  (uint8_t)(addr >> 16),
                                  (uint8_t)(addr >> 8),
                                  (uint8_t)addr };
                cs_low();
                spi_tx(cmd, 4);
                cs_high();
            }

            flash_wait_busy();
            addr += 65536;
            blocks_erased++;
        } else {
            // 4KB sector erase for remainder
            uint8_t wren = CMD_WREN;
            cs_low();
            spi_tx(&wren, 1);
            cs_high();

            if (use_4byte) {
                uint8_t cmd[5] = { CMD_SE_4K_4B,
                                  (uint8_t)(addr >> 24),
                                  (uint8_t)(addr >> 16),
                                  (uint8_t)(addr >> 8),
                                  (uint8_t)addr };
                cs_low();
                spi_tx(cmd, 5);
                cs_high();
            } else {
                uint8_t cmd[4] = { CMD_SE_4K,
                                  (uint8_t)(addr >> 16),
                                  (uint8_t)(addr >> 8),
                                  (uint8_t)addr };
                cs_low();
                spi_tx(cmd, 4);
                cs_high();
            }

            flash_wait_busy();
            addr += 4096;
        }

        if (blocks_erased % 64 == 0 && blocks_erased > 0) {
            printf("[PROGRESS] Erased %.2f MB\n",
                   (float)addr / (1024.0f * 1024.0f));
        }
    }

    return true;
}

// ============================================================================
// Restore Function (with 4-byte addressing support)
// ============================================================================
/**
 * Restores flash contents from SD card backup file
 * Supports chips up to 4GB with automatic 4-byte addressing
 */
bool flash_restore_from_sd_with_size(const char *filename, uint32_t known_size) {
    printf("\n[RESTORE] Starting flash restore...\n");
    printf("[WARNING] This will ERASE and REPROGRAM the entire flash chip!\n");
    printf("[INFO] Using chip size: %lu bytes (%.2f MB)\n",
           known_size, (float)known_size / (1024.0f * 1024.0f));

    // Determine if 4-byte addressing is needed
    bool use_4byte = (known_size > 16 * 1024 * 1024);
    if (use_4byte) {
        printf("[INFO] Chip >16MB: Enabling 4-byte addressing mode\n");
        flash_enter_4byte_mode();
    }

    // Step 1: Open backup file
    FIL file;
    FRESULT fr = f_open(&file, filename, FA_READ);
    if (fr != FR_OK) {
        printf("[ERROR] Failed to open backup file\n");
        if (use_4byte) flash_exit_4byte_mode();
        return false;
    }

    DWORD file_size = f_size(&file);
    printf("[INFO] Backup file size: %lu bytes\n", file_size);

    uint32_t restore_size = (file_size < known_size) ? file_size : known_size;

    // Step 2: Remove write protection
    printf("[INFO] Removing write protection...\n");
    flash_global_unprotect();

    // Step 3: Smart erase
    printf("[INFO] Erasing chip...\n");
    if (!flash_smart_erase(restore_size, known_size, use_4byte)) {
        printf("[ERROR] Erase failed!\n");
        f_close(&file);
        if (use_4byte) flash_exit_4byte_mode();
        return false;
    }

    // Step 4: Program pages
    printf("[INFO] Programming flash from backup...\n");
    uint8_t page[256];
    uint32_t addr = 0;
    uint32_t pages_programmed = 0;
    uint32_t total_pages = (restore_size + 255) / 256;

    f_lseek(&file, 0);

    while (addr < restore_size) {
        // Read page from SD
        UINT bytes_read = 0;
        memset(page, 0xFF, sizeof(page));
        fr = f_read(&file, page, sizeof(page), &bytes_read);
        if (fr != FR_OK) {
            printf("[ERROR] SD read failed at %lu\n", addr);
            f_close(&file);
            if (use_4byte) flash_exit_4byte_mode();
            return false;
        }

        if (bytes_read == 0) break;

        // Write enable
        uint8_t wren = CMD_WREN;
        cs_low();
        spi_tx(&wren, 1);
        cs_high();

        // Page program with appropriate addressing
        if (use_4byte) {
            uint8_t cmd[5] = { CMD_PP_4B,
                              (uint8_t)(addr >> 24),
                              (uint8_t)(addr >> 16),
                              (uint8_t)(addr >> 8),
                              (uint8_t)addr };
            cs_low();
            spi_tx(cmd, 5);
            spi_tx(page, 256);
            cs_high();
        } else {
            uint8_t cmd[4] = { CMD_PP,
                              (uint8_t)(addr >> 16),
                              (uint8_t)(addr >> 8),
                              (uint8_t)addr };
            cs_low();
            spi_tx(cmd, 4);
            spi_tx(page, 256);
            cs_high();
        }

        flash_wait_busy();
        addr += 256;
        pages_programmed++;

        // Progress indicator
        if (pages_programmed % 256 == 0 || addr >= restore_size) {
            printf("[PROGRESS] %lu/%lu pages (%.1f%%)\n",
                   pages_programmed, total_pages,
                   (float)pages_programmed * 100.0f / (float)total_pages);
        }
    }

    f_close(&file);
    if (use_4byte) flash_exit_4byte_mode();

    printf("[SUCCESS] Restore complete: %lu pages (%lu bytes)\n",
           pages_programmed, addr);
    return true;
}

/**
 * Legacy restore function (tries SFDP detection first)
 */
bool flash_restore_from_sd(const char *filename) {
    uint32_t chip_size = flash_detect_size();
    if (chip_size == 0) {
        printf("[WARNING] Using backup file size...\n");
        FIL file;
        if (f_open(&file, filename, FA_READ) != FR_OK) {
            printf("[ERROR] Failed to open backup file\n");
            return false;
        }
        chip_size = f_size(&file);
        f_close(&file);
    }
    return flash_restore_from_sd_with_size(filename, chip_size);
}

/**
 * Verifies flash contents against backup file
 */
bool flash_verify_from_sd(const char *filename) {
    printf("\n[VERIFY] Starting verification...\n");

    FIL file;
    if (f_open(&file, filename, FA_READ) != FR_OK) {
        printf("[ERROR] Failed to open backup file\n");
        return false;
    }

    DWORD file_size = f_size(&file);
    printf("[INFO] Verifying %lu bytes...\n", file_size);

    // Determine if 4-byte addressing is needed
    bool use_4byte = (file_size > 16 * 1024 * 1024);
    if (use_4byte) {
        flash_enter_4byte_mode();
    }

    uint8_t flash_buf[4096];
    uint8_t file_buf[4096];
    uint32_t addr = 0;
    bool match = true;

    while (addr < file_size) {
        uint32_t chunk_size = (file_size - addr > sizeof(flash_buf)) ?
                             sizeof(flash_buf) : (file_size - addr);

        // Read from flash with appropriate addressing
        if (use_4byte) {
            uint8_t cmd[5] = { CMD_READ,  // Can use standard read for verify
                              (uint8_t)(addr >> 24),
                              (uint8_t)(addr >> 16),
                              (uint8_t)(addr >> 8),
                              (uint8_t)addr };
            cs_low();
            spi_tx(cmd, 5);
            spi_rx(flash_buf, chunk_size);
            cs_high();
        } else {
            uint8_t cmd[4] = { CMD_READ,
                              (uint8_t)(addr >> 16),
                              (uint8_t)(addr >> 8),
                              (uint8_t)addr };
            cs_low();
            spi_tx(cmd, 4);
            spi_rx(flash_buf, chunk_size);
            cs_high();
        }

        // Read from file
        UINT bytes_read = 0;
        if (f_read(&file, file_buf, chunk_size, &bytes_read) != FR_OK ||
            bytes_read != chunk_size) {
            printf("[ERROR] SD read failed\n");
            match = false;
            break;
        }

        // Compare
        if (memcmp(flash_buf, file_buf, chunk_size) != 0) {
            printf("[ERROR] Verification failed at offset %lu!\n", addr);
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

        if (addr % (256 * 1024) == 0) {
            printf("[PROGRESS] Verified %.2f MB\n",
                   (float)addr / (1024.0f * 1024.0f));
        }
    }

    f_close(&file);
    if (use_4byte) flash_exit_4byte_mode();

    if (match) {
        printf("[SUCCESS] Verification passed!\n");
    }

    return match;
}
