/**
 * @file main_flash_forensic.c
 *
 * @brief SPI Flash Performance Evaluation & Forensic Analysis System
 *
 * This module implements a dual-core architecture where Core 0 performs
 * SPI flash benchmarking operations and Core 1 handles web interface
 * and data export functionality.
 *
 * @par Hardware Requirements
 * - Raspberry Pi Pico W (Master) with WiFi capability
 * - Raspberry Pi Pico (Slave) for direct SPI operations
 * - ROBO-PICO carrier boards with microSD card slots
 * - SPI flash chips for testing
 *
 * @par Copyright
 * (c) 2024 Embedded Systems Project
 */

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/rtc.h"
#include "hardware/spi.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "helper_functions/read.h"
#include "helper_functions/write.h"
#include "helper_functions/erase.h"
#include "helper_functions/sd_functions.h"
#include "helper_functions/identification.h"
#include "helper_functions/display_functions.h"


// ============================================================================
// Constants and Configuration
// ============================================================================

#define BUTTON_START_PIN      20
#define BUTTON_DISPLAY_PIN    21
#define DEBOUNCE_DELAY_MS     50
#define STARTUP_DELAY_MS      7000
#define MAX_MOUNT_ATTEMPTS    3
#define MOUNT_RETRY_DELAY_MS  500
#define POST_MOUNT_DELAY_MS   200

// ============================================================================
// Private Data Types
// ============================================================================

typedef enum
{
    STATE_INIT = 0,
    STATE_READY,
    STATE_BENCHMARKING,
    STATE_IDENTIFYING,
    STATE_LOGGING,
    STATE_ERROR
} system_state_t;

typedef struct
{
    system_state_t state;
    bool sd_mounted;
    bool database_loaded;
    uint32_t benchmark_count;
    uint32_t last_button_time;
    bool last_button_state;
} system_context_t;

// ============================================================================
// Private Global Variables
// ============================================================================

static system_context_t g_system_ctx;
static volatile bool g_core1_ready = false;
static volatile bool g_benchmark_request = false;
static volatile bool g_benchmark_complete = false;

// ============================================================================
// Private Function Prototypes
// ============================================================================

static void system_init(void);
static void core0_main_loop(void);
static void core1_entry(void);
static void core1_main_loop(void);
static bool button_is_pressed(uint32_t pin);
static void handle_benchmark_button(void);
static void handle_display_button(void);
static void run_full_benchmark_cycle(void);
static void print_system_banner(void);
static void print_startup_instructions(void);

// ============================================================================
// Public Function Implementations
// ============================================================================

/**
 * @brief Main entry point for the application
 *
 * Initializes all system components and launches dual-core operation.
 * Core 0 handles benchmarking and user interface, Core 1 handles
 * web server and data export.
 *
 * @return Never returns (infinite loop)
 */
int
main(void)
{
    system_init();
    
    print_system_banner();
    
    // Launch Core 1 for web server and export operations
    multicore_launch_core1(core1_entry);
    
    // Wait for Core 1 initialization
    while (!g_core1_ready)
    {
        tight_loop_contents();
    }
    
    printf("[SYSTEM] Dual-core initialization complete\n");
    print_startup_instructions();
    
    // Core 0 main loop handles benchmarking and UI
    core0_main_loop();
    
    return 0;
}

// ============================================================================
// Private Function Implementations
// ============================================================================

/**
 * @brief Initialize all system components
 *
 * Performs hardware initialization, SD card mounting, and database loading.
 */
static void
system_init(void)
{
    // Initialize stdio
    stdio_init_all();
    sleep_ms(STARTUP_DELAY_MS);
    
    // Initialize RTC
    datetime_t rtc_time = {
        .year = 2024,
        .month = 1,
        .day = 1,
        .dotw = 1,
        .hour = 0,
        .min = 0,
        .sec = 0
    };
    rtc_init();
    rtc_set_datetime(&rtc_time);
    
    // Initialize system context
    memset(&g_system_ctx, 0, sizeof(g_system_ctx));
    g_system_ctx.state = STATE_INIT;
    g_system_ctx.last_button_state = true;
    
    // Initialize GPIO buttons
    gpio_init(BUTTON_START_PIN);
    gpio_set_dir(BUTTON_START_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_START_PIN);
    
    gpio_init(BUTTON_DISPLAY_PIN);
    gpio_set_dir(BUTTON_DISPLAY_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_DISPLAY_PIN);
    
    // Initialize SPI flash interface
    spi_flash_init();
    
    // Initialize SD card and load database
    int32_t mount_attempts = 0;
    while (!g_system_ctx.sd_mounted && (mount_attempts < MAX_MOUNT_ATTEMPTS))
    {
        printf("[SD] Mount attempt %d/%d...\n", 
               mount_attempts + 1, MAX_MOUNT_ATTEMPTS);
        
        if (sd_card_mount() == 0)
        {
            g_system_ctx.sd_mounted = true;
            printf("[SD] Mount successful\n");
            sleep_ms(POST_MOUNT_DELAY_MS);
            
            // Load chip database
            if (database_load() == 0)
            {
                g_system_ctx.database_loaded = true;
                printf("[DATABASE] Loaded %d chip entries\n", 
                       database_get_entry_count());
            }
        }
        else
        {
            printf("[SD] Mount failed\n");
            mount_attempts++;
            if (mount_attempts < MAX_MOUNT_ATTEMPTS)
            {
                sleep_ms(MOUNT_RETRY_DELAY_MS);
            }
        }
    }
    
    if (!g_system_ctx.sd_mounted)
    {
        printf("[WARNING] SD card not available - limited functionality\n");
    }
    
    g_system_ctx.state = STATE_READY;
}

/**
 * @brief Core 0 main processing loop
 *
 * Handles user button inputs and coordinates benchmarking operations.
 * This core performs all SPI flash operations and chip identification.
 */
static void
core0_main_loop(void)
{
    for (;;)
    {
        uint32_t current_time = to_ms_since_boot(get_absolute_time());
        
        // Check benchmark start button (GP20)
        if (button_is_pressed(BUTTON_START_PIN))
        {
            if ((current_time - g_system_ctx.last_button_time) > DEBOUNCE_DELAY_MS)
            {
                handle_benchmark_button();
                g_system_ctx.last_button_time = current_time;
            }
        }
        
        // Check display button (GP21)
        if (button_is_pressed(BUTTON_DISPLAY_PIN))
        {
            if ((current_time - g_system_ctx.last_button_time) > DEBOUNCE_DELAY_MS)
            {
                handle_display_button();
                g_system_ctx.last_button_time = current_time;
            }
        }
        
        sleep_ms(10);
    }
}

/**
 * @brief Core 1 entry point
 *
 * Initializes Core 1 components and enters main loop.
 */
static void
core1_entry(void)
{
    // Initialize web server
    web_server_init();
    
    // Signal Core 0 that initialization is complete
    g_core1_ready = true;
    
    // Enter Core 1 main loop
    core1_main_loop();
}

/**
 * @brief Core 1 main processing loop
 *
 * Handles web server operations and data export functionality.
 */
static void
core1_main_loop(void)
{
    for (;;)
    {
        // Process web server requests
        web_server_poll();
        
        // Check for benchmark completion and export data
        if (g_benchmark_complete)
        {
            web_server_update_results();
            g_benchmark_complete = false;
        }
        
        sleep_ms(10);
    }
}

/**
 * @brief Check if button is currently pressed
 *
 * @param[in] pin GPIO pin number to check
 * @return true if button pressed (active low), false otherwise
 */
static bool
button_is_pressed(uint32_t pin)
{
    if (!gpio_get(pin))
    {
        sleep_ms(20);  // Debounce delay
        return (!gpio_get(pin));
    }
    return false;
}

/**
 * @brief Handle benchmark start button press
 *
 * Initiates a complete benchmark cycle including identification,
 * performance testing, and data logging.
 */
static void
handle_benchmark_button(void)
{
    printf("\n[BUTTON] Benchmark requested (GP20)\n");
    
    // Verify system is ready
    if (!g_system_ctx.database_loaded)
    {
        printf("[ERROR] Database not loaded - cannot perform identification\n");
        return;
    }
    
    g_system_ctx.state = STATE_BENCHMARKING;
    run_full_benchmark_cycle();
    g_system_ctx.state = STATE_READY;
    
    // Signal Core 1 that new results are available
    g_benchmark_complete = true;
}

/**
 * @brief Handle display button press
 *
 * Shows the current chip database contents via serial output.
 */
static void
handle_display_button(void)
{
    printf("\n[BUTTON] Display database requested (GP21)\n");
    
    if (!g_system_ctx.database_loaded)
    {
        printf("[ERROR] Database not loaded\n");
        return;
    }
    
    database_print_all();
}

/**
 * @brief Execute complete benchmark and identification cycle
 *
 * This function performs the following operations:
 * 1. Chip identification via JEDEC ID and SFDP
 * 2. Read/write/erase performance benchmarking
 * 3. Multi-factor database matching
 * 4. Results logging to SD card
 */
static void
run_full_benchmark_cycle(void)
{
    printf("\n");
    printf("========================================\n");
    printf("BENCHMARK CYCLE START\n");
    printf("========================================\n");
    
    // Step 1: Identify chip
    printf("\n[STEP 1/4] Chip Identification\n");
    chip_ident_t chip_id;
    if (spi_flash_identify(&chip_id) != 0)
    {
        printf("[ERROR] Chip identification failed\n");
        return;
    }
    
    // Step 2: Run performance benchmarks
    printf("\n[STEP 2/4] Performance Benchmarks\n");
    benchmark_results_t bench_results;
    
    // Read benchmarks at multiple clock speeds
    benchmark_run_read_tests(&bench_results);
    
    // Write benchmarks
    benchmark_run_write_tests(&bench_results);
    
    // Erase benchmarks
    benchmark_run_erase_tests(&bench_results);
    
    // Step 3: Database matching
    printf("\n[STEP 3/4] Database Matching\n");
    match_result_t match_results[5];  // Top 5 matches
    identification_match_database(&chip_id, &bench_results, match_results, 5);
    
    // Step 4: Log results
    printf("\n[STEP 4/4] Results Logging\n");
    if (g_system_ctx.sd_mounted)
    {
        sd_card_log_results(&chip_id, &bench_results, match_results, 5);
        sd_card_create_forensic_report(&chip_id, &bench_results);
    }
    else
    {
        printf("[WARNING] SD card not available - results not logged\n");
    }
    
    // Print summary
    printf("\n");
    printf("========================================\n");
    printf("BENCHMARK CYCLE COMPLETE\n");
    printf("========================================\n");
    identification_print_results(match_results, 5);
    
    g_system_ctx.benchmark_count++;
}

/**
 * @brief Print system startup banner
 */
static void
print_system_banner(void)
{
    printf("\n");
    printf("========================================\n");
    printf("SPI FLASH FORENSIC ANALYSIS SYSTEM\n");
    printf("========================================\n");
    printf("Dual-Core Architecture:\n");
    printf("  Core 0: Benchmarking & Identification\n");
    printf("  Core 1: Web Interface & Data Export\n");
    printf("========================================\n");
}

/**
 * @brief Print startup instructions for user
 */
static void
print_startup_instructions(void)
{
    printf("\n");
    printf("SYSTEM READY\n");
    printf("----------------------------------------\n");
    printf("GP20 Button: Start Benchmark Cycle\n");
    printf("GP21 Button: Display Database\n");
    printf("----------------------------------------\n");
    printf("\n");
}

/*** end of file ***/