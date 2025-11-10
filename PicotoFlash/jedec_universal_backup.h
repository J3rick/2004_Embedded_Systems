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

// SPI + pin configuration
typedef struct {
    spi_inst_t *spi;
    uint cs_pin;
    uint wp_pin;
    uint hold_pin;
    uint sck_pin;
    uint mosi_pin;
    uint miso_pin;
    uint32_t clk_hz;
} jedec_bus_t;

// Chip properties discovered by probe
typedef struct {
    uint8_t manuf_id;
    uint8_t mem_type;
    uint8_t capacity_id;

    uint32_t total_bytes;
    bool has_sfdp;
    bool use_4byte_addr;

    uint32_t page_size;
    uint32_t sector_size;

    uint8_t read_cmd;
    uint8_t dummy_cycles;

    uint32_t effective_spi_hz;
} jedec_chip_t;

// Sink callback: receives each read block
typedef bool (*jedec_sink_cb)(
    const uint8_t *data,
    size_t len,
    uint32_t offset,
    void *user
);

// Init SPI + pins
bool jedec_init(const jedec_bus_t *bus);

// JEDEC ID + SFDP detection + safe read settings
bool jedec_probe(jedec_chip_t *out);

// Read a region into sink
bool jedec_backup_stream(
    const jedec_chip_t *chip,
    uint32_t offset,
    uint32_t len,
    size_t chunk,
    jedec_sink_cb sink,
    void *user
);

// Read whole chip
bool jedec_backup_full(
    const jedec_chip_t *chip,
    jedec_sink_cb sink,
    void *user
);

// Internal helper: read a chunk into RAM
bool jedec_read_chunk(
    const jedec_chip_t *chip,
    uint32_t addr,
    uint8_t *buf,
    size_t len
);

#ifdef __cplusplus
}
#endif

#endif
