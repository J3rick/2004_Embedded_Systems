/*
 * Universal JEDEC Flash Backup Module (Read-only)
 *
 * - JEDEC ID (0x9F)
 * - Optional SFDP detection (0x5A)
 * - Auto-select safe read command:
 *      * 0x03   – always safe
 *      * 0x0B   – fast read if SFDP says flash supports it
 * - 3-byte vs 4-byte addressing
 * - Full-chip or partial backup via callback sink
 */

#include "jedec_universal_backup.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

static jedec_bus_t g_bus;

// === SPI helpers ===
static inline void cs_low(void)  { gpio_put(g_bus.cs_pin, 0); }
static inline void cs_high(void) { gpio_put(g_bus.cs_pin, 1); }

static inline void spi_tx(const uint8_t *buf, size_t len) {
    spi_write_blocking(g_bus.spi, buf, len);
}
static inline void spi_rx(uint8_t *buf, size_t len) {
    spi_read_blocking(g_bus.spi, 0x00, buf, len);
}

// === Init SPI + pins ===
bool jedec_init(const jedec_bus_t *bus) {
    g_bus = *bus;

    gpio_init(g_bus.cs_pin);
    gpio_set_dir(g_bus.cs_pin, GPIO_OUT);
    cs_high();

    if (g_bus.wp_pin != 0xFFFFFFFF) {
        gpio_init(g_bus.wp_pin);
        gpio_set_dir(g_bus.wp_pin, GPIO_OUT);
        gpio_put(g_bus.wp_pin, 1);
    }
    if (g_bus.hold_pin != 0xFFFFFFFF) {
        gpio_init(g_bus.hold_pin);
        gpio_set_dir(g_bus.hold_pin, GPIO_OUT);
        gpio_put(g_bus.hold_pin, 1);
    }

    gpio_set_function(g_bus.sck_pin,  GPIO_FUNC_SPI);
    gpio_set_function(g_bus.mosi_pin, GPIO_FUNC_SPI);
    gpio_set_function(g_bus.miso_pin, GPIO_FUNC_SPI);
    gpio_pull_up(g_bus.miso_pin);

    spi_init(g_bus.spi, g_bus.clk_hz);

    return true;
}

// === JEDEC ID ===
static void read_jedec_id(uint8_t id[3]) {
    uint8_t cmd = 0x9F;
    cs_low();
    spi_tx(&cmd, 1);
    spi_rx(id, 3);
    cs_high();
}

// === Try reading SFDP signature ===
static bool try_sfdp(uint8_t *buf, size_t len) {
    uint8_t hdr[5] = {0x5A, 0, 0, 0, 0};
    cs_low();
    spi_tx(hdr, 5);
    spi_rx(buf, len);
    cs_high();
    return (len >= 4 &&
            buf[0] == 'S' &&
            buf[1] == 'F' &&
            buf[2] == 'D' &&
            buf[3] == 'P');
}

// === Convert capacity_id to bytes (fallback when no SFDP) ===
static uint32_t capacity_from_id(uint8_t cap_id){
    if (cap_id >= 32) return 0;              // guard overflow for 32-bit
    return 1u << cap_id;                     // bytes
}

// === Probe flash characteristics ===
bool jedec_probe(jedec_chip_t *out) {
    memset(out, 0, sizeof *out);

    // effective SPI actual frequency
    out->effective_spi_hz = spi_set_baudrate(g_bus.spi, g_bus.clk_hz);

    uint8_t id[3] = {0};
    read_jedec_id(id);

    out->manuf_id    = id[0];
    out->mem_type    = id[1];
    out->capacity_id = id[2];

    uint8_t sfdp_buf[16] = {0};
    out->has_sfdp = try_sfdp(sfdp_buf, sizeof sfdp_buf);

    // fallback capacity if no SFDP
    out->total_bytes = capacity_from_id(out->capacity_id);
    if (out->total_bytes == 0)
        out->total_bytes = 512 * 1024; // sanity minimum

    // 4-byte addressing for >16MiB
    out->use_4byte_addr = (out->total_bytes > 16u * 1024u * 1024u);

    if (out->use_4byte_addr) {
        uint8_t cmd = 0xB7; // ENTER 4-BYTE ADDRESS MODE
        cs_low();
        spi_tx(&cmd, 1);
        cs_high();
    }

    // conservative defaults
    out->page_size   = 256;
    out->sector_size = 4096;

    // read command selection
    if (out->has_sfdp) {
        out->read_cmd     = 0x0B; // fast read
        out->dummy_cycles = 8;
    } else {
        out->read_cmd     = 0x03; // safe read
        out->dummy_cycles = 0;
    }

    return true;
}

// === Low-level chunk read ===
bool jedec_read_chunk(const jedec_chip_t *chip, uint32_t addr, uint8_t *buf, size_t len) {
    uint8_t hdr[6];
    size_t h = 0;

    hdr[h++] = chip->read_cmd;

    if (chip->use_4byte_addr) {
        hdr[h++] = (addr >> 24) & 0xFF;
    }
    hdr[h++] = (addr >> 16) & 0xFF;
    hdr[h++] = (addr >> 8)  & 0xFF;
    hdr[h++] = (addr >> 0)  & 0xFF;

    if (chip->read_cmd == 0x0B) {
        hdr[h++] = 0x00; // dummy
    }

    cs_low();
    spi_tx(hdr, h);
    spi_rx(buf, len);
    cs_high();
    return true;
}

// === Backup (streamed through callback) ===
bool jedec_backup_stream(
    const jedec_chip_t *chip,
    uint32_t offset,
    uint32_t len,
    size_t chunk,
    jedec_sink_cb sink,
    void *user
) {
    if (!sink || chunk == 0)
        return false;

    uint8_t *buf = malloc(chunk);
    if (!buf)
        return false;

    uint32_t end = offset + len;

    for (uint32_t a = offset; a < end; ) {
        size_t n = chunk;
        if (a + n > end)
            n = end - a;

        if (!jedec_read_chunk(chip, a, buf, n)) {
            free(buf);
            return false;
        }

        if (!sink(buf, n, a, user)) {
            free(buf);
            return false;
        }

        a += n;
        tight_loop_contents();
    }

    free(buf);
    return true;
}

// === Backup entire flash ===
bool jedec_backup_full(
    const jedec_chip_t *chip,
    jedec_sink_cb sink,
    void *user
) {
    return jedec_backup_stream(
        chip,
        0,
        chip->total_bytes,
        64 * 1024,   // 64KiB blocks
        sink,
        user
    );
}
