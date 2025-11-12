#ifndef JEDEC_UNIVERSAL_BACKUP_H
#define JEDEC_UNIVERSAL_BACKUP_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"

#ifdef __cplusplus
extern "C" {
#endif



// ===== Bus + chip structs =====
typedef struct {
    spi_inst_t *spi;
    uint cs_pin;
    uint wp_pin;    // 0xFFFFFFFF if not wired
    uint hold_pin;  // 0xFFFFFFFF if not wired
    uint sck_pin;
    uint mosi_pin;
    uint miso_pin;
    uint32_t clk_hz;
} jedec_bus_t;

typedef struct {
    uint8_t  manuf_id;
    uint8_t  mem_type;
    uint8_t  capacity_id;

    uint32_t total_bytes;
    bool     has_sfdp;
    bool     use_4byte_addr;

    uint32_t page_size;   // default 256
    uint32_t sector_size; // default 4096

    uint8_t  read_cmd;      // 0x03 or 0x0B
    uint8_t  dummy_cycles;  // for 0x0B fast read

    uint32_t effective_spi_hz;
} jedec_chip_t;

// ===== Stream callbacks =====
typedef bool (*jedec_sink_cb)(const uint8_t *data, size_t len, uint32_t offset, void *user);       // backup → user
typedef bool (*jedec_source_cb)(uint8_t *dest, size_t want, size_t *got, uint32_t offset, void *user); // user → restore

// ===== Probe / init =====
bool jedec_init(const jedec_bus_t *bus);
bool jedec_probe(jedec_chip_t *out);

// ===== Backup (read) =====
bool jedec_read_chunk(const jedec_chip_t *chip, uint32_t addr, uint8_t *buf, size_t len);
bool jedec_backup_stream(const jedec_chip_t *chip, uint32_t offset, uint32_t len,
                         size_t chunk, jedec_sink_cb sink, void *user);
bool jedec_backup_full(const jedec_chip_t *chip, jedec_sink_cb sink, void *user);


// ===== Restore (erase + program) =====
typedef struct {
    bool verify_after_write;    // default true
    bool skip_erase_when_all_ff;// default true
    bool skip_prog_when_all_ff; // default true
    uint32_t program_chunk;     // default 256 (page)
    uint32_t erase_gran;        // default 4096 (sector)
} jedec_restore_opts_t;

bool jedec_try_unprotect(const jedec_chip_t *chip); // best-effort generic unprotect
bool jedec_erase_range(const jedec_chip_t *chip, uint32_t addr, uint32_t len, uint32_t erase_gran);

bool jedec_restore_stream(const jedec_chip_t *chip, uint32_t offset, uint32_t len,
                          jedec_source_cb src, void *user, const jedec_restore_opts_t *opts);
bool jedec_restore_full(const jedec_chip_t *chip, jedec_source_cb src, void *user,
                        const jedec_restore_opts_t *opts);

#ifdef __cplusplus
}
#endif

#endif
