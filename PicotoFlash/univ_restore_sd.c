#include "univ_restore_sd.h"
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "ff.h"

// ---------------- low-level SPI helpers (3-byte addressing path) ----------------
static inline void cs_low (const jedec_bus_t* b){ gpio_put(b->cs_pin, 0); }
static inline void cs_high(const jedec_bus_t* b){ gpio_put(b->cs_pin, 1); }

static inline void spi_tx(const jedec_bus_t* b, const uint8_t* buf, size_t n) {
    spi_write_blocking((spi_inst_t*)b->spi, buf, n);
}
static inline void spi_rx(const jedec_bus_t* b, uint8_t* buf, size_t n) {
    spi_read_blocking((spi_inst_t*)b->spi, 0x00, buf, n);
}

// --- standard commands (operate on the provided bus) ---
static inline void wren(const jedec_bus_t* b) {
    uint8_t cmd = 0x06; cs_low(b); spi_tx(b, &cmd, 1); cs_high(b);
}
static inline void we_for_sr(const jedec_bus_t* b) { // ok if unsupported
    uint8_t cmd = 0x50; cs_low(b); spi_tx(b, &cmd, 1); cs_high(b);
}
static inline uint8_t rd_sr1(const jedec_bus_t* b) {
    uint8_t cmd = 0x05, v = 0; cs_low(b); spi_tx(b, &cmd, 1); spi_rx(b, &v, 1); cs_high(b); return v;
}
static inline void wait_busy(const jedec_bus_t* b) {
    while (rd_sr1(b) & 0x01) tight_loop_contents();
}

// Clear protection bits in SR1 (BPx=0). Works for tiny parts too.
static void clear_bp_sr1(const jedec_bus_t* b) {
    we_for_sr(b);              // write-enable for SR (ignored if N/A)
    wren(b);                   // standard write enable
    uint8_t wr[2] = {0x01, 0x00};  // WRSR + SR1 value
    cs_low(b); spi_tx(b, wr, 2); cs_high(b);
    wait_busy(b);

    // Optional vendor global-unlock (ignored by chips that don't support it)
    uint8_t gul = 0x98;
    cs_low(b); spi_tx(b, &gul, 1); cs_high(b);
}

// 4KB sector erase (0x20) — 3-byte addressing
static void erase_4k(const jedec_bus_t* b, uint32_t addr) {
    wren(b);
    uint8_t h[4] = {0x20, (uint8_t)(addr>>16), (uint8_t)(addr>>8), (uint8_t)addr};
    cs_low(b); spi_tx(b, h, 4); cs_high(b);
    wait_busy(b);
}

// Page program (<=256B) — 3-byte addressing
static void page_program(const jedec_bus_t* b, uint32_t addr, const uint8_t* p, size_t n) {
    wren(b);
    uint8_t h[4] = {0x02, (uint8_t)(addr>>16), (uint8_t)(addr>>8), (uint8_t)addr};
    cs_low(b); spi_tx(b, h, 4); spi_tx(b, p, n); cs_high(b);
    wait_busy(b);
}

// Slow, safe verify read (0x03) — avoids fast-read dummy pitfalls
static void read_03(const jedec_bus_t* b, uint32_t addr, uint8_t* buf, size_t n) {
    uint8_t h[4] = {0x03, (uint8_t)(addr>>16), (uint8_t)(addr>>8), (uint8_t)addr};
    cs_low(b); spi_tx(b, h, 4); spi_rx(b, buf, n); cs_high(b);
}

// ---------------- universal restore from SD image (full-chip) ----------------
bool universal_restore_from_sd(const char *path,
                               const jedec_bus_t *bus,
                               bool verify_after_write)
{
    if (!path || !bus) { printf("[RESTORE] bad args\n"); return false; }

    // Probe current chip on this bus
    jedec_chip_t chip = {0};
    jedec_init(bus);
    jedec_probe(&chip);

    if (chip.total_bytes == 0) {
        printf("[RESTORE] probe failed (size=0)\n");
        return false;
    }

    // This implementation supports 3-byte addressing (<= 16 MiB)
    if (chip.total_bytes > (16u<<20)) {
        printf("[RESTORE] >16MiB not supported by this build (size=%u)\n", chip.total_bytes);
        return false;
    }

    // Open SD input
    FIL f;
    FRESULT fr = f_open(&f, path, FA_READ);
    if (fr != FR_OK) {
        printf("[RESTORE] open %s failed (%d)\n", path, (int)fr);
        return false;
    }

    // Size check
    FSIZE_t fsz = f_size(&f);
    if ((uint32_t)fsz != chip.total_bytes) {
        printf("[RESTORE] size mismatch: file=%llu, chip=%u\n",
               (unsigned long long)fsz, chip.total_bytes);
        f_close(&f);
        return false;
    }

    // Unprotect (SR1-only) — robust across tiny JEDEC parts
    uint8_t sr1_before = rd_sr1(bus);
    clear_bp_sr1(bus);
    uint8_t sr1_after  = rd_sr1(bus);
    printf("[UNPROTECT] SR1=0x%02X -> 0x%02X\n", sr1_before, sr1_after);

    // Program by 4KB sectors
    const uint32_t SECT = 4096;
    const uint32_t PAGE = 256;
    uint8_t buf[PAGE];
    uint32_t off = 0;
    UINT br = 0;

    printf("[RESTORE] Restoring %u bytes from %s...\n", chip.total_bytes, path);

    while (off < chip.total_bytes) {
        uint32_t sec_base = off;                  // start of this sector in the stream
        erase_4k(bus, off & ~(SECT - 1));         // erase target sector at flash address

        // program pages inside this 4KB sector
        for (uint32_t p = 0; p < SECT; p += PAGE) {
            uint32_t addr = (off & ~(SECT - 1)) + p;
            if (addr >= chip.total_bytes) break;

            br = 0;
            fr = f_read(&f, buf, PAGE, &br);
            if (fr != FR_OK) {
                printf("[RESTORE] f_read err=%d @0x%08X\n", (int)fr, addr);
                f_close(&f);
                return false;
            }
            if (br == 0) break; // EOF guard (should not happen if size matched)

            page_program(bus, addr, buf, br);
            if (br < PAGE) break; // last partial page for the last bytes
        }

        off = sec_base + SECT;

        if ((off & ((1u<<18)-1u)) == 0) { // every 256 KiB
            printf("[RESTORE] %u / %u written...\n", off, chip.total_bytes);
        }
    }

    // Optional verify pass
    f_lseek(&f, 0);
    if (verify_after_write) {
        printf("[RESTORE] PROGRAM DONE. Verifying...\n");
        uint8_t sd_buf[PAGE];
        uint8_t rd_buf[PAGE];
        off = 0;

        while (off < chip.total_bytes) {
            UINT want = PAGE;
            if (off + want > chip.total_bytes) want = chip.total_bytes - off;

            br = 0;
            fr = f_read(&f, sd_buf, want, &br);
            if (fr != FR_OK || br != want) {
                printf("[RESTORE] verify SD read err=%d br=%u @0x%08X\n",
                       (int)fr, (unsigned)br, off);
                f_close(&f);
                return false;
            }

            read_03(bus, off, rd_buf, want);

            for (UINT i = 0; i < want; i++) {
                if (rd_buf[i] != sd_buf[i]) {
                    uint32_t bad = off + i;
                    printf("[RESTORE] VERIFY MISMATCH at 0x%08X (sector+%u): wrote 0x%02X read 0x%02X\n",
                           bad, (unsigned)(bad & 0xFFF), sd_buf[i], rd_buf[i]);
                    f_close(&f);
                    return false;
                }
            }

            off += want;
            if ((off & ((1u<<20)-1u)) == 0) {
                printf("[RESTORE] VERIFY %u / %u OK...\n", off, chip.total_bytes);
            }
            tight_loop_contents();
        }
        printf("[RESTORE] VERIFY OK (byte-for-byte)\n");
    } else {
        printf("[RESTORE] PROGRAM DONE. Verify skipped.\n");
    }

    f_close(&f);
    return true;
}
