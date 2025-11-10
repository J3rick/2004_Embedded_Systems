// unified_flash_benchmark.c
// Single Pico benchmarks its own onboard flash (W25Q16JV typically)
// Press GP20 button to start benchmarking

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "hardware/clocks.h"
#include "read.h"
#include "erase.h"
#include "write.h"

// ---------- SPI Flash pins (to onboard W25Q flash) ----------
#define FLASH_SPI spi0
#define PIN_SCK   2
#define PIN_MOSI  3
#define PIN_MISO  4
#define PIN_CS    6

// ---------- Button ----------
#define PIN_BUTTON  20  // Button to GND (active-low, pull-up enabled)

// Destructive erase bench
#define ENABLE_DESTRUCTIVE_TESTS 1

// Safe test window (avoid factory-protected bottom sector and bootloader)
#define TEST_BASE_ADDR   0x100000u  // 1MB offset (safe area)
#define PAGE_SIZE        256u

// ---------- Chip Database Structure ----------
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
    float    read_speed_50mhz_mbs;
} chip_db_entry_t;

// ---------- Chip Database ----------
static const chip_db_entry_t chip_database[] = {
    // Winbond W25Q series (common on Pico boards)
    {0xEF4015, "W25Q16JV", "Winbond", "W25Q", 16, 45, 400, 120, 1000, 150, 2000, 133, 0, 3, 0},
    {0xEF4016, "W25Q32", "Winbond", "W25Q", 32, 45, 400, 120, 1000, 150, 2000, 133, 0, 3, 0},
    {0xEF4017, "W25Q64", "Winbond", "W25Q", 64, 45, 400, 120, 1000, 150, 2000, 133, 0, 3, 0},
    {0xEF4018, "W25Q128", "Winbond", "W25Q", 128, 45, 400, 120, 1000, 150, 2000, 133, 0, 3, 0},
    {0xEF4019, "W25Q256", "Winbond", "W25Q", 256, 45, 400, 120, 1000, 150, 2000, 133, 0, 3, 0},

    // Macronix MX25L series
    {0xC22016, "MX25L3233F", "Macronix", "MX25L", 32, 45, 240, 120, 600, 150, 1200, 133, 0, 5, 0},
    {0xC22017, "MX25L6433F", "Macronix", "MX25L", 64, 45, 240, 120, 600, 150, 1200, 133, 0, 5, 0},
    {0xC22018, "MX25L12833F", "Macronix", "MX25L", 128, 45, 240, 120, 600, 150, 1200, 133, 0, 5, 0},

    // GigaDevice GD25Q series
    {0xC84016, "GD25Q32", "GigaDevice", "GD25Q", 32, 50, 400, 150, 2000, 200, 3000, 120, 0, 5, 0},
    {0xC84017, "GD25Q64", "GigaDevice", "GD25Q", 64, 50, 400, 150, 2000, 200, 3000, 120, 0, 5, 0},
    {0xC84018, "GD25Q128", "GigaDevice", "GD25Q", 128, 50, 400, 150, 2000, 200, 3000, 120, 0, 5, 0},

    // ISSI IS25LP series
    {0x9D6016, "IS25LP032", "ISSI", "IS25LP", 32, 25, 130, 60, 300, 70, 400, 133, 0, 3, 0},
    {0x9D6017, "IS25LP064", "ISSI", "IS25LP", 64, 25, 130, 60, 300, 70, 400, 133, 0, 3, 0},
    {0x9D6018, "IS25LP128", "ISSI", "IS25LP", 128, 25, 130, 60, 300, 70, 400, 133, 0, 3, 0},

    // ISSI IS25WP series
    {0x9D7016, "IS25WP032", "ISSI", "IS25WP", 32, 25, 130, 60, 300, 70, 400, 133, 0, 3, 0},
    {0x9D7017, "IS25WP064", "ISSI", "IS25WP", 64, 25, 130, 60, 300, 70, 400, 133, 0, 3, 0},

    // ISSI IS25LQ series (older)
    {0x9D4013, "IS25LP040E", "ISSI", "IS25LP", 4, 70, 300, 130, 500, 200, 1000, 104, 1, 2, 0},
    {0x9D4014, "IS25LQ080B", "ISSI", "IS25LQ", 8, 30, 150, 80, 400, 100, 500, 104, 0, 3, 0},
    {0x9D4015, "IS25LQ016B", "ISSI", "IS25LQ", 16, 30, 150, 80, 400, 100, 500, 104, 0, 3, 0},
    {0x9D4016, "IS25LQ032B", "ISSI", "IS25LQ", 32, 30, 150, 80, 400, 100, 500, 104, 0, 3, 0},

    // Micron/ST M25P series
    {0x202016, "M25P32", "Micron", "M25P", 32, 0, 3000, 0, 40000, 0, 40000, 75, 0, 5, 0},
    {0x202017, "M25P64", "Micron", "M25P", 64, 0, 3000, 0, 40000, 0, 40000, 75, 0, 5, 0},
};

#define CHIP_DB_SIZE (sizeof(chip_database) / sizeof(chip_database[0]))

// ---------- Button helpers ----------
static void init_button(void) {
    gpio_init(PIN_BUTTON);
    gpio_set_dir(PIN_BUTTON, GPIO_IN);
    gpio_pull_up(PIN_BUTTON);
}

static bool button_pressed(void) {
    if (!gpio_get(PIN_BUTTON)) { 
        sleep_ms(20); 
        return !gpio_get(PIN_BUTTON); 
    }
    return false;
}

// ---------- SPI helpers ----------
static inline void cs_low(void){ gpio_put(PIN_CS, 0); }
static inline void cs_high(void){ gpio_put(PIN_CS, 1); }
static inline void spi_tx(const uint8_t *b, size_t n){ spi_write_blocking(FLASH_SPI, b, n); }
static inline void spi_rx(uint8_t *b, size_t n){ spi_read_blocking(FLASH_SPI, 0x00, b, n); }
static uint32_t bytes_for_n(uint8_t n){ return (n<32)? (1u<<n):0u; }

// ---------- Flash basics ----------
static void read_jedec_id(uint8_t out[3]){ 
    uint8_t c=0x9F; 
    cs_low(); 
    spi_tx(&c,1); 
    spi_rx(out,3); 
    cs_high(); 
}

static bool read_sfdp(uint32_t a, uint8_t *buf, size_t n){
    if(a>0xFFFFFF) return false;
    uint8_t h[5]={0x5A,(uint8_t)(a>>16),(uint8_t)(a>>8),(uint8_t)a,0};
    cs_low(); spi_tx(h,5); spi_rx(buf,n); cs_high(); return true;
}

static void flash_read03(uint32_t addr, uint8_t *buf, size_t len){
    uint8_t h[4]={0x03,(uint8_t)(addr>>16),(uint8_t)(addr>>8),(uint8_t)addr};
    cs_low(); spi_tx(h,4); spi_rx(buf,len); cs_high();
}

// ---------- Identify + SFDP ----------
typedef struct {
    uint8_t jedec[3];
    bool sfdp_ok; 
    uint8_t sfdp_major,sfdp_minor; 
    uint32_t density_bits;
    bool et_present[4]; 
    uint8_t et_opcode[4]; 
    uint32_t et_size_bytes[4];
    bool fast_read_0B; 
    uint8_t fast_read_dummy;
} ident_t;

static void identify(ident_t *id){
    memset(id,0,sizeof(*id));
    read_jedec_id(id->jedec);

    uint32_t saved = spi_get_baudrate(FLASH_SPI); 
    spi_set_baudrate(FLASH_SPI, 5*1000*1000);
    uint8_t hdr[8]={0};
    if(read_sfdp(0, hdr, 8) && hdr[0]=='S'&&hdr[1]=='F'&&hdr[2]=='D'&&hdr[3]=='P'){
        id->sfdp_ok = true; 
        id->sfdp_minor = hdr[4]; 
        id->sfdp_major = hdr[5];
        uint8_t nph=(uint8_t)(hdr[6]+1);
        uint8_t ph[8*16]={0}; 
        size_t need=(size_t)nph*8; 
        if(need>sizeof(ph)) need=sizeof(ph);
        read_sfdp(8, ph, need);

        bool bfpt=false; 
        uint32_t ptp=0; 
        uint8_t dwords=0;
        for(uint8_t i=0;i<nph;i++){
            const uint8_t *p=&ph[i*8];
            uint16_t idlsb=(uint16_t)p[0]|((uint16_t)p[1]<<8);
            if(idlsb==0xFF00 || idlsb==0x00FF){ 
                bfpt=true; 
                ptp=(uint32_t)p[5]|((uint32_t)p[6]<<8)|((uint32_t)p[7]<<16); 
                dwords=p[4]; 
                break; 
            }
        }
        if(!bfpt){ 
            ptp=0x000030; 
            dwords=64; 
            bfpt=true; 
        }

        if(bfpt){
            size_t bytes=(size_t)(dwords? dwords*4:256); 
            if(bytes>256) bytes=256;
            uint8_t bf[256]={0};
            if(read_sfdp(ptp, bf, bytes)){
                if(bytes>=8){
                    uint32_t d2=(uint32_t)bf[4]|((uint32_t)bf[5]<<8)|((uint32_t)bf[6]<<16)|((uint32_t)bf[7]<<24);
                    printf("  [SFDP] DENSITY_DWORD2=%08X\n", d2);
                    if((d2&0x80000000u)==0) id->density_bits = d2 + 1u;
                    else{ 
                        uint32_t n=(d2&0x7FFFFFFFu)+1u; 
                        if(n<32) id->density_bits=(1u<<n); 
                    }
                }
                if(bytes>=32){
                    uint32_t d7=(uint32_t)bf[24]|((uint32_t)bf[25]<<8)|((uint32_t)bf[26]<<16)|((uint32_t)bf[27]<<24);
                    uint32_t d8=(uint32_t)bf[28]|((uint32_t)bf[29]<<8)|((uint32_t)bf[30]<<16)|((uint32_t)bf[31]<<24);
                    uint8_t szn[4]={(uint8_t)(d7>>0),(uint8_t)(d7>>16),(uint8_t)(d8>>0),(uint8_t)(d8>>16)};
                    uint8_t opc[4]={(uint8_t)(d7>>8),(uint8_t)(d7>>24),(uint8_t)(d8>>8),(uint8_t)(d8>>24)};
                    for(int k=0;k<4;k++){ 
                        uint32_t sz=bytes_for_n(szn[k]); 
                        id->et_present[k]=(sz!=0); 
                        id->et_opcode[k]=opc[k]; 
                        id->et_size_bytes[k]=sz; 
                    }
                }
            }
        }
    }
    
    // Probe 0x0B works
    { 
        uint8_t c[5]={0x0B,0,0,0,0}, v=0xA5; 
        cs_low(); 
        spi_tx(c,5); 
        spi_rx(&v,1); 
        cs_high(); 
        id->fast_read_0B = true; 
        id->fast_read_dummy = 1; 
        (void)v; 
    }
    spi_set_baudrate(FLASH_SPI, saved);
}

// ---------- Database Lookup ----------
static const chip_db_entry_t* lookup_chip(uint32_t jedec_id){
    for(size_t i = 0; i < CHIP_DB_SIZE; i++){
        if(chip_database[i].jedec_id == jedec_id){
            return &chip_database[i];
        }
    }
    return NULL;
}

// ---------- Capacity probe ----------
static void capacity_probe(void){
    static uint8_t a[256], b[256];
    const uint32_t offs[] = {512*1024, 1024*1024, 2*1024*1024};
    flash_read03(0x000000, a, sizeof a);
    for(size_t i=0;i<sizeof(offs)/sizeof(offs[0]);++i){
        flash_read03(offs[i], b, sizeof b);
        bool same = (memcmp(a,b,sizeof a)==0);
        printf("  [CAP_PROBE] 0x%06X : %s\n", offs[i], same? "MIRRORS":"DIFF");
    }
}

// ---------- Pretty printing helpers ----------
static void print_divider(int width){
    for(int i=0;i<width;i++) putchar('-');
    putchar('\n');
}

static void print_section(const char* title){
    putchar('\n');
    print_divider(72);
    printf("%s\n", title);
    print_divider(72);
}

static void print_kv(const char* k, const char* v){
    printf("  %-18s : %s\n", k, v);
}

static void print_kv_hex24(const char* k, uint8_t a, uint8_t b, uint8_t c){
    printf("  %-18s : %02X%02X%02X\n", k, a,b,c);
}

static uint32_t mbit_from_ident_capacity(const ident_t* id, const chip_db_entry_t* chip){
    if (chip && chip->capacity_mbit) return chip->capacity_mbit;
    if (id->density_bits){
        return (uint32_t)((id->density_bits + 500000u) / 1000000u);
    }
    return 0;
}

static void print_supported_cmds(const ident_t* id){
    bool has4=false,has32=false,has64=false; 
    uint8_t o4=0,o32=0,o64=0;
    for(int k=0;k<4;k++){
        if(!id->et_present[k]) continue;
        if(id->et_size_bytes[k]==4096){  has4=true;  o4=id->et_opcode[k]; }
        if(id->et_size_bytes[k]==32768){  has32=true; o32=id->et_opcode[k]; }
        if(id->et_size_bytes[k]==65536){  has64=true; o64=id->et_opcode[k]; }
    }
    char buf[128]; 
    buf[0]='\0';
    int n=0;
    n+=snprintf(buf+n, sizeof(buf)-n, "READ 0x03");
    if (id->fast_read_0B) n+=snprintf(buf+n, sizeof(buf)-n, ", FAST_READ 0x0B (dummy=%u)", id->fast_read_dummy? id->fast_read_dummy:1);
    n+=snprintf(buf+n, sizeof(buf)-n, ", PAGE_PROG 0x02");
    if (has4)  n+=snprintf(buf+n, sizeof(buf)-n, ", ERASE4K 0x%02X", o4);
    if (has32) n+=snprintf(buf+n, sizeof(buf)-n, ", ERASE32K 0x%02X", o32);
    if (has64) n+=snprintf(buf+n, sizeof(buf)-n, ", ERASE64K 0x%02X", o64);
    print_kv("Supported cmds", buf);
}

static void print_overview(const ident_t* id, const chip_db_entry_t* chip){
    print_section("CHIP IDENTIFICATION");
    print_kv_hex24("JEDEC ID", id->jedec[0], id->jedec[1], id->jedec[2]);
    uint32_t mbit = mbit_from_ident_capacity(id, chip);
    if (mbit){
        printf("  %-18s : %u Mbit (%u MBytes)\n", "Capacity", mbit, (unsigned)(mbit/8));
    }else{
        print_kv("Capacity", "(unknown)");
    }
    char sfdp[32]; 
    snprintf(sfdp, sizeof sfdp, "%s v%u.%u", id->sfdp_ok? "yes":"no", id->sfdp_major, id->sfdp_minor);
    print_kv("SFDP present", sfdp);

    if (chip){
        char model[96];
        snprintf(model, sizeof model, "%s (%s %s), max %u MHz",
                 chip->model, chip->company, chip->family, chip->max_clock_mhz);
        print_kv("Database", "FOUND");
        print_kv("Chip", model);
    }else{
        print_kv("Database", "NOT FOUND");
    }
    print_supported_cmds(id);
    print_divider(72);
}

// ---------- Main ----------
int main(void){
    stdio_init_all();
    sleep_ms(7000);

    printf("\n========================================\n");
    printf("UNIFIED FLASH BENCHMARK\n");
    printf("========================================\n");
    printf("System clock: %u Hz\n", (unsigned)clock_get_hz(clk_sys));
    printf("Peripheral clock: %u Hz\n", (unsigned)clock_get_hz(clk_peri));
    printf("\nPress GP20 button to start benchmarking...\n");
    printf("========================================\n\n");

    spi_init(FLASH_SPI, 5*1000*1000);

    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_init(PIN_CS); 
    gpio_set_dir(PIN_CS, GPIO_OUT); 
    cs_high();

    init_button();

    for(;;){
        // Wait for button press
        while(!button_pressed()) {
            tight_loop_contents();
        }

        printf("\n[BUTTON PRESSED] Starting benchmark...\n");
        sleep_ms(100);
        
        // Reset results storage
        read_reset_results();
        erase_reset_results();

        // Identify chip (but don't print yet)
        ident_t id; 
        memset(&id,0,sizeof id);
        identify(&id);

        // Database lookup
        uint32_t jedec_id = ((uint32_t)id.jedec[0] << 16) | ((uint32_t)id.jedec[1] << 8) | id.jedec[2];
        const chip_db_entry_t *chip = lookup_chip(jedec_id);

        // Run read benchmarks at multiple clocks (now runs once per clock)
        bool use_fast = id.fast_read_0B;
        uint8_t dummy = use_fast ? (id.fast_read_dummy ? id.fast_read_dummy : 1) : 0;
        const int clock_list[] = {63, 32, 21, 16, 13};
        const int NCLK = (int)(sizeof(clock_list)/sizeof(clock_list[0]));
        read_bench_capture_t caps[NCLK]; 
        memset(caps, 0, sizeof(caps));

        for (int i = 0; i < NCLK; i++) {
            int mhz = clock_list[i];
            printf("  [FORCED_CLK] typ_clk=%d, mode=%s, dummy=%u\n",
                   mhz, use_fast ? "0x0B" : "0x03", dummy);
            read_run_benches_capture(FLASH_SPI, PIN_CS, use_fast, dummy, mhz, &caps[i]);
        }

        read_derive_and_print_50(clock_list, caps, NCLK);

#if ENABLE_DESTRUCTIVE_TESTS
        // Write benchmarks section
        print_section("WRITE/PROGRAM BENCHMARKS");
        printf("This test WRITES data to flash in test region.\n");
        printf("Testing page programming performance at various data sizes...\n\n");
        
        const int write_clocks[] = {21, 16};
        const int num_write_clocks = sizeof(write_clocks) / sizeof(write_clocks[0]);
        write_bench_capture_t write_captures[num_write_clocks];
        
        int write_success = write_bench_run_multi_clock(
            FLASH_SPI,
            PIN_CS,
            write_clocks,
            num_write_clocks,
            TEST_BASE_ADDR + 0x10000,
            write_captures
        );
        
        printf("\n[INFO] Completed %d/%d write benchmark runs\n", 
               write_success, num_write_clocks);
        
        if (write_success > 0) {
            write_bench_print_summary(write_captures, num_write_clocks);
        }
#else
        print_section("WRITE BENCHMARKS DISABLED");
#endif

        // Erase benchmarks (now runs once per erase type)
        print_section("WRITE/ERASE SAFETY NOTICE");
        printf("This test ERASES flash in a test region at 0x%06X.\n", TEST_BASE_ADDR);
        
        // Convert ident_t to erase_ident_t
        erase_ident_t erase_id;
        memcpy(&erase_id, &id, sizeof(erase_ident_t));
        
        // Convert chip_db_entry_t to erase_chip_db_entry_t (they're compatible)
        const erase_chip_db_entry_t *erase_chip = (const erase_chip_db_entry_t *)chip;
        
        erase_flash_unprotect(FLASH_SPI, PIN_CS, id.jedec[0], TEST_BASE_ADDR);

#if ENABLE_DESTRUCTIVE_TESTS
        const int ERASE_FIXED_MHZ = 21;
        erase_run_benches_at_clock(FLASH_SPI, PIN_CS, &erase_id, erase_chip, 
                                   ERASE_FIXED_MHZ, TEST_BASE_ADDR);
#else
        print_section("ERASE BENCHMARKS DISABLED");
#endif

        print_section("BENCHMARKS COMPLETE");
        
        // Print summary tables
        read_print_summary_tables();
        erase_print_summary_tables();
        
        // NOW print chip identification at the end
        capacity_probe();
        print_overview(&id, chip);
        
        printf("\nPress GP20 again to rerun benchmarks...\n");

        // Wait for button release
        while(!gpio_get(PIN_BUTTON)) {
            tight_loop_contents();
        }
        sleep_ms(150);
    }

    return 0;
}