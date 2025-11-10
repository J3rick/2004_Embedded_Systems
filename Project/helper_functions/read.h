#ifndef READ_H
#define READ_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "hardware/spi.h"

// Read sizes configuration
#define NUM_READ_SIZES 5
extern const size_t k_read_sizes[NUM_READ_SIZES];
extern const char* k_read_labels[NUM_READ_SIZES];

// Statistics structure
typedef struct {
    double avg_us, p25, p50, p75;
    uint32_t vmin, vmax;
    double std_us, mb_s;
} read_stats_t;

// Benchmark row structure
typedef struct {
    size_t size_bytes;
    read_stats_t stats;
} read_bench_row_t;

// Benchmark capture structure
typedef struct {
    int actual_mhz;
    read_bench_row_t rows[NUM_READ_SIZES];
    bool filled;
} read_bench_capture_t;

// Read result storage
typedef struct {
    int clock_mhz;
    bool valid;
    read_stats_t size_stats[NUM_READ_SIZES];
} read_result_t;

// Global results storage
extern read_result_t g_read_results[8];
extern int g_read_result_count;

// Function declarations
void read_reset_results(void);
void read_save_result(int mhz, const read_bench_capture_t *cap);
void read_run_benches_capture(spi_inst_t *spi, uint8_t cs_pin, bool use_fast, 
                              uint8_t dummy, int mhz_req, read_bench_capture_t *cap_out);
void read_derive_and_print_50(const int *req_mhz, const read_bench_capture_t *caps, int n_caps);
void read_print_summary_tables(void);

#endif // READ_H