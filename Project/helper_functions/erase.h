#ifndef ERASE_H
#define ERASE_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/spi.h"

// Erase block sizes
#define SECTOR_4K   4096u
#define BLOCK_32K   32768u
#define BLOCK_64K   65536u

// Erase result storage
typedef struct {
    int clock_mhz;
    bool valid;
    double avg_4k, avg_32k, avg_64k;
    uint32_t min_4k, max_4k;
    uint32_t min_32k, max_32k;
    uint32_t min_64k, max_64k;
} erase_result_t;

// Identification structure (needed for erase type detection)
typedef struct {
    uint8_t jedec[3];
    bool sfdp_ok;
    uint8_t sfdp_major, sfdp_minor;
    uint32_t density_bits;
    bool et_present[4];
    uint8_t et_opcode[4];
    uint32_t et_size_bytes[4];
    bool fast_read_0B;
    uint8_t fast_read_dummy;
} erase_ident_t;

// Chip database entry (for timing info)
typedef struct {
    uint32_t jedec_id;
    const char *model;
    const char *company;
    const char *family;
    uint32_t capacity_mbit;
    uint16_t typ_4kb_erase_ms;
    uint16_t max_4kb_erase_ms;
    uint16_t typ_32kb_erase_ms;
    uint16_t max_32kb_erase_ms;
    uint16_t typ_64kb_erase_ms;
    uint16_t max_64kb_erase_ms;
    uint16_t max_clock_mhz;
    uint16_t typ_page_prog_ms;
    uint16_t max_page_prog_ms;
    float read_speed_50mhz_mbs;
} erase_chip_db_entry_t;

// Global results storage
extern erase_result_t g_erase_result;

// Function declarations
void erase_reset_results(void);
void erase_save_result(int mhz, double avg4k, uint32_t min4k, uint32_t max4k,
                       double avg32k, uint32_t min32k, uint32_t max32k,
                       double avg64k, uint32_t min64k, uint32_t max64k);
void erase_run_benches_at_clock(spi_inst_t *spi, uint8_t cs_pin,
                                const erase_ident_t *id,
                                const erase_chip_db_entry_t *chip,
                                int mhz, uint32_t test_addr);
void erase_print_summary_tables(void);
void erase_flash_unprotect(spi_inst_t *spi, uint8_t cs_pin, uint8_t mfr, uint32_t test_addr);

#endif // ERASE_H