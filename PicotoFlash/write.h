// flash_write_bench.h
// Write/Program benchmark functions for SPI flash

#ifndef FLASH_WRITE_BENCH_H
#define FLASH_WRITE_BENCH_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

// Write benchmark configuration
#define WRITE_ITERS_DEFAULT 10
#define WRITE_TEST_SIZES 5

// Statistics structure for write benchmarks
typedef struct {
    double avg_us;
    double p25, p50, p75;
    uint32_t vmin, vmax;
    double std_us;
    double mb_s;
} write_stats_t;

// Write benchmark result for one size
typedef struct {
    size_t size_bytes;
    const char *label;
    write_stats_t stats;
    bool verify_ok;
} write_bench_result_t;

// Complete write benchmark results for one clock speed
typedef struct {
    int clock_mhz_requested;
    int clock_mhz_actual;
    bool valid;
    write_bench_result_t results[WRITE_TEST_SIZES];
    int num_results;
} write_bench_capture_t;

// Function prototypes

/**
 * Run write benchmarks at specified clock speed
 * 
 * @param spi_inst SPI instance (e.g., spi0)
 * @param cs_pin Chip select GPIO pin
 * @param mhz_req Requested SPI clock in MHz
 * @param base_addr Base address for test (should be in safe area, e.g., 0x100000)
 * @param sizes Array of sizes to test (in bytes)
 * @param labels Array of labels for each size
 * @param num_sizes Number of sizes to test
 * @param iterations Number of iterations per size
 * @param capture Output structure to store results
 * @return true if successful, false on error
 */
bool write_bench_run(void *spi_inst, uint8_t cs_pin, int mhz_req, 
                     uint32_t base_addr, const size_t *sizes, 
                     const char **labels, int num_sizes, 
                     int iterations, write_bench_capture_t *capture);

/**
 * Run write benchmarks at multiple clock speeds
 * 
 * @param spi_inst SPI instance
 * @param cs_pin Chip select GPIO pin
 * @param clocks Array of clock speeds to test (MHz)
 * @param num_clocks Number of clock speeds
 * @param base_addr Base address for test
 * @param captures Output array to store results (must have space for num_clocks)
 * @return Number of successful benchmark runs
 */
int write_bench_run_multi_clock(void *spi_inst, uint8_t cs_pin,
                                const int *clocks, int num_clocks,
                                uint32_t base_addr,
                                write_bench_capture_t *captures);

/**
 * Print write benchmark results table
 * 
 * @param capture Benchmark results to print
 */
void write_bench_print_results(const write_bench_capture_t *capture);

/**
 * Print summary table comparing multiple clock speeds
 * 
 * @param captures Array of benchmark results
 * @param num_captures Number of results
 */
void write_bench_print_summary(const write_bench_capture_t *captures, int num_captures);

/**
 * Print performance summary (MB/s) for multiple clocks
 * 
 * @param captures Array of benchmark results
 * @param num_captures Number of results
 */
void write_bench_print_performance_summary(const write_bench_capture_t *captures, int num_captures);

/**
 * Print timing summary (microseconds) for multiple clocks
 * 
 * @param captures Array of benchmark results
 * @param num_captures Number of results
 */
void write_bench_print_timing_summary(const write_bench_capture_t *captures, int num_captures);

#endif // FLASH_WRITE_BENCH_H