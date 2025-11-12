#include "jedec_universal_backup.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

static jedec_bus_t g_bus;

// ===== Low-level helpers =====
static inline void cs_low(void)  { gpio_put(g_bus.cs_pin, 0); }
static inline void cs_high(void) { gpio_put(g_bus.cs_pin, 1); }

static inline void spi_tx(const uint8_t *buf, size_t len){
    spi_write_blocking(g_bus.spi, buf, len);
}
static inline void spi_rx(uint8_t *buf, size_t len){
    spi_read_blocking(g_bus.spi, 0x00, buf, len);
}

// --- write/erase helpers ---
static void wren(void){
    uint8_t c = 0x06;
    cs_low(); spi_tx(&c,1); cs_high();
}
static uint8_t rd_status1(void){
    uint8_t c=0x05, v=0;
    cs_low(); spi_tx(&c,1); spi_rx(&v,1); cs_high();
    return v;
}
static void wait_busy(void){
    while (rd_status1() & 0x01) { sleep_ms(1); }
}
static void wr_status1_1b(uint8_t s1){
    wren();
    uint8_t h[2] = {0x01, s1};
    cs_low(); spi_tx(h,2); cs_high();
    wait_busy();
}
static void wr_status1_2b(uint8_t s1, uint8_t s2){
    wren();
    uint8_t h[3] = {0x01, s1, s2};
    cs_low(); spi_tx(h,3); cs_high();
    wait_busy();
}

// ===== Init / probe =====
static void read_jedec_id(uint8_t id[3]){
    uint8_t cmd = 0x9F;
    cs_low(); spi_tx(&cmd,1); spi_rx(id,3); cs_high();
}

static bool try_sfdp(uint8_t *buf, size_t len){
    uint8_t cmd[5] = {0x5A, 0,0,0,0};
    cs_low(); spi_tx(cmd,5); spi_rx(buf,len); cs_high();
    return (len>=4 && buf[0]=='S' && buf[1]=='F' && buf[2]=='D' && buf[3]=='P');
}

// Treat capacity_id as log2(BYTES): e.g., 0x16→2^22 B = 4MiB
static uint32_t capacity_from_id(uint8_t cap_id){
    if (cap_id >= 32) return 0;
    return 1u << cap_id;
}

bool jedec_init(const jedec_bus_t *bus){
    g_bus = *bus;

    gpio_init(g_bus.cs_pin);
    gpio_set_dir(g_bus.cs_pin, GPIO_OUT);
    cs_high();

    if (g_bus.wp_pin != 0xFFFFFFFF){
        gpio_init(g_bus.wp_pin);
        gpio_set_dir(g_bus.wp_pin, GPIO_OUT);
        gpio_put(g_bus.wp_pin, 1);
    }
    if (g_bus.hold_pin != 0xFFFFFFFF){
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

bool jedec_probe(jedec_chip_t *out){
    memset(out,0,sizeof *out);
    out->effective_spi_hz = spi_set_baudrate(g_bus.spi, g_bus.clk_hz);

    uint8_t id[3]={0}; read_jedec_id(id);
    out->manuf_id=id[0]; out->mem_type=id[1]; out->capacity_id=id[2];

    uint8_t sfdp_hdr[16]={0};
    out->has_sfdp = try_sfdp(sfdp_hdr, sizeof sfdp_hdr);

    out->total_bytes = capacity_from_id(out->capacity_id);
    if (out->total_bytes==0) out->total_bytes = 512*1024; // conservative fallback

    out->use_4byte_addr = (out->total_bytes > 16u*1024u*1024u);
    if (out->use_4byte_addr){ uint8_t c=0xB7; cs_low(); spi_tx(&c,1); cs_high(); } // EN4B

    out->page_size   = 256;
    out->sector_size = 4096;

    if (out->has_sfdp){ out->read_cmd=0x0B; out->dummy_cycles=8; }
    else              { out->read_cmd=0x03; out->dummy_cycles=0; }

    return true;
}

// ===== Backup (read) =====
bool jedec_read_chunk(const jedec_chip_t *chip, uint32_t addr, uint8_t *buf, size_t len){
    uint8_t hdr[6]; size_t h=0;
    hdr[h++]=chip->read_cmd;

    if (chip->use_4byte_addr) hdr[h++]=(addr>>24)&0xFF;
    hdr[h++]=(addr>>16)&0xFF; hdr[h++]=(addr>>8)&0xFF; hdr[h++]=(addr)&0xFF;

    if (chip->read_cmd==0x0B) hdr[h++]=0x00; // dummy

    cs_low(); spi_tx(hdr,h); spi_rx(buf,len); cs_high();
    return true;
}

bool jedec_backup_stream(const jedec_chip_t *chip, uint32_t offset, uint32_t len,
                         size_t chunk, jedec_sink_cb sink, void *user){
    if (!sink || chunk==0) return false;
    uint8_t *buf = (uint8_t*)malloc(chunk); if (!buf) return false;

    uint32_t end = offset + len;
    for (uint32_t a = offset; a < end; ){
        size_t n = chunk; if (a+n> end) n = end-a;
        if (!jedec_read_chunk(chip,a,buf,n)){ free(buf); return false; }
        if (!sink(buf,n,a,user))           { free(buf); return false; }
        a += n; tight_loop_contents();
    }
    free(buf); return true;
}

bool jedec_backup_full(const jedec_chip_t *chip, jedec_sink_cb sink, void *user){
    return jedec_backup_stream(chip, 0, chip->total_bytes, 64*1024, sink, user);
}

// ===== Restore (erase + program + verify) =====
static void erase_4k(const jedec_chip_t *chip, uint32_t addr){
    (void)chip;
    wren();
    uint8_t h[5]; size_t n=0;
    h[n++]=0x20; // 4KiB erase
    if (chip->use_4byte_addr) h[n++]=(addr>>24)&0xFF;
    h[n++]=(addr>>16)&0xFF; h[n++]=(addr>>8)&0xFF; h[n++]=(addr)&0xFF;
    cs_low(); spi_tx(h,n); cs_high();
    wait_busy();
}

static void prog_page(const jedec_chip_t *chip, uint32_t addr, const uint8_t *data, size_t len){
    // len <= page_size, does not cross page boundary
    (void)chip;
    wren();
    uint8_t h[5]; size_t n=0;
    h[n++]=0x02; // page program
    if (chip->use_4byte_addr) h[n++]=(addr>>24)&0xFF;
    h[n++]=(addr>>16)&0xFF; h[n++]=(addr>>8)&0xFF; h[n++]=(addr)&0xFF;
    cs_low(); spi_tx(h,n); spi_tx(data,len); cs_high();
    wait_busy();
}

// ---- Status helpers ----
static uint8_t rd_status2(void){
    uint8_t c=0x35, v=0;
    cs_low(); spi_tx(&c,1); spi_rx(&v,1); cs_high();
    return v;
}
static uint8_t rd_status3(void){
    uint8_t c=0x15, v=0;
    cs_low(); spi_tx(&c,1); spi_rx(&v,1); cs_high();
    return v;
}

// Some parts want "Enable Write SR" prior to WRSR (legacy)
// Safe to try; ignored by many modern chips.
static void ewrsr_legacy(void){
    uint8_t c = 0x50; // EWSR (Write Enable for Status Register) - legacy
    cs_low(); spi_tx(&c,1); cs_high();
}


bool jedec_try_unprotect(const jedec_chip_t *chip){
    (void)chip;

    // Read current status
    uint8_t sr1_before = rd_status1();
    uint8_t sr2_before = rd_status2(); // QE lives here on Winbond
    uint8_t sr3_before = rd_status3();

    printf("[UNPROTECT] SR1=0x%02X SR2=0x%02X SR3=0x%02X\n",
           sr1_before, sr2_before, sr3_before);

    // 1) Global Block Unlock (Winbond/GD/ISSI support this). Safe to try.
    {
        uint8_t c = 0x98; // ULBPR / Global Unlock
        wren();           // some parts require WREN first
        cs_low(); spi_tx(&c,1); cs_high();
        wait_busy();
    }

    // 2) Clear BP bits in SR1 while preserving other bits; preserve QE in SR2.
    //    Winbond BP bits: SR1[2:6] (BP0..BP4). Also clear SRP0 if set (bit7) so SR writes are allowed.
    uint8_t sr1_new = sr1_before & ~( (1u<<2) | (1u<<3) | (1u<<4) | (1u<<5) | (1u<<6) | (1u<<7) );
    uint8_t sr2_new = sr2_before; // keep QE as-is

    // Some chips require EWSR (legacy) before WRSR; safe to attempt.
    ewrsr_legacy();

    // Winbond accepts 0x01 with two data bytes (SR1,SR2). Many chips ignore the 2nd byte if unsupported.
    wren();
    {
        uint8_t h[3] = {0x01, sr1_new, sr2_new};
        cs_low(); spi_tx(h,3); cs_high();
        wait_busy();
    }

    // Optional: write SR3 back unchanged on families that support it (0x11)
    // Not strictly necessary; comment out if you prefer.
    // wren();
    // { uint8_t h[2] = {0x11, sr3_before}; cs_low(); spi_tx(h,2); cs_high(); wait_busy(); }

    // Read back to confirm
    uint8_t sr1_after = rd_status1();
    uint8_t sr2_after = rd_status2();
    uint8_t sr3_after = rd_status3();

    printf("[UNPROTECT] After: SR1=0x%02X SR2=0x%02X SR3=0x%02X\n",
           sr1_after, sr2_after, sr3_after);

    bool bp_cleared =
        ((sr1_after & ((1u<<2)|(1u<<3)|(1u<<4)|(1u<<5)|(1u<<6))) == 0);

    if (!bp_cleared){
        printf("[UNPROTECT] WARN: BP bits not fully cleared; device may still be locked.\n");
    }
    return true; // return true even if partial, so caller continues and verify will tell us
}


static inline bool all_ff(const uint8_t *p, size_t n){
    for (size_t i=0;i<n;i++) if (p[i]!=0xFF) return false; return true;
}

bool jedec_erase_range(const jedec_chip_t *chip, uint32_t addr, uint32_t len, uint32_t erase_gran){
    if (erase_gran==0) erase_gran = chip->sector_size;
    uint32_t end = addr + len;
    for (uint32_t a = addr; a < end; a += erase_gran){
        erase_4k(chip, a);
    }
    return true;
}

bool jedec_restore_stream(const jedec_chip_t *chip, uint32_t offset, uint32_t len,
                          jedec_source_cb src, void *user, const jedec_restore_opts_t *opts_in){
    if (!src) return false;
    jedec_restore_opts_t opts = {
        .verify_after_write = true,
        .skip_erase_when_all_ff = true,
        .skip_prog_when_all_ff  = true,
        .program_chunk = chip->page_size ? chip->page_size : 256,
        .erase_gran    = chip->sector_size ? chip->sector_size : 4096
    };
    if (opts_in) opts = *opts_in;

    uint32_t sector = opts.erase_gran;
    uint8_t *buf   = (uint8_t*)malloc(sector);
    uint8_t *vbuf  = (uint8_t*)malloc(sector);
    if (!buf || !vbuf){ free(buf); free(vbuf); return false; }

    jedec_try_unprotect(chip);

    uint32_t end = offset + len;
    for (uint32_t base = offset; base < end; base += sector){
        size_t want = sector;
        if (base + want > end) want = end - base;

        size_t got = 0;
        if (!src(buf, want, &got, base, user) || got != want){
            free(buf); free(vbuf); return false;
        }

        bool skip_sector = opts.skip_erase_when_all_ff && all_ff(buf, want);
        if (!skip_sector){
            // erase sector
            erase_4k(chip, base);

            // program by pages
            for (uint32_t off = 0; off < want; ){
                uint32_t page_addr   = base + off;
                uint32_t page_off    = page_addr & (opts.program_chunk - 1);
                uint32_t this_left   = opts.program_chunk - page_off;
                uint32_t remain      = want - off;
                uint32_t n           = (this_left < remain) ? this_left : remain;

                if (!(opts.skip_prog_when_all_ff && all_ff(&buf[off], n))){
                    prog_page(chip, page_addr, &buf[off], n);
                }
                off += n;
            }

            if (opts.verify_after_write){
                if (!jedec_read_chunk(chip, base, vbuf, want)){
                    printf("[RESTORE] Readback failed at 0x%08X\n", base);
                    free(buf); free(vbuf); return false;
                }
                if (memcmp(buf, vbuf, want)!=0){
                    // Find first mismatching byte for precise diagnosis
                    size_t bad = 0;
                    while (bad < want && buf[bad] == vbuf[bad]) bad++;
                    printf("[RESTORE] VERIFY MISMATCH at 0x%08X (sector+%u): wrote 0x%02X read 0x%02X\n",
                        base, (unsigned)bad, buf[bad], vbuf[bad]);
                    free(buf); free(vbuf);
                    return false;
                }   
}

        }
        tight_loop_contents();
    }

    free(buf); free(vbuf);
    return true;
}

bool jedec_restore_full(const jedec_chip_t *chip, jedec_source_cb src, void *user,
                        const jedec_restore_opts_t *opts){
    return jedec_restore_stream(chip, 0, chip->total_bytes, src, user, opts);
}
