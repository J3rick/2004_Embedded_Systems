/*
 * Flash Chip Identification Module
 * Contains chip matching and confidence calculation functions
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "identification.h"

// External references to global data from main.c
extern FlashChipData database[];
extern int database_entry_count;
extern FlashChipData benchmark_results;
extern match_result_t match_results[];

// ============================================================================
// D7.2.3: chip_calculate_confidence()
// ============================================================================

confidence_result_t chip_calculate_confidence(FlashChipData* measured, FlashChipData* expected) {
    confidence_result_t result;
    memset(&result, 0, sizeof(confidence_result_t));
    
    // Adjusted weighting - SKIP Write (Page Program) and Clock
    const float JEDEC_WEIGHT = 0.40;
    const float READ_WEIGHT = 0.20;
    const float ERASE_WEIGHT = 0.10;
    // Write (20%) and Clock (10%) are SKIPPED - not included
    
    // Tolerances
    const float READ_TOLERANCE = 0.15;
    const float ERASE_TOLERANCE = 0.20;
    const float MEASUREMENT_UNCERTAINTY = 0.05;
    
    int factors_available = 0;
    float total_weight_available = 0.0;
    float weighted_score = 0.0;
    
    // 1. JEDEC ID Match (40% weight)
    if (strlen(measured->jedec_id) > 0 && strlen(expected->jedec_id) > 0) {
        result.breakdown.jedec_id_available = true;
        factors_available++;
        total_weight_available += JEDEC_WEIGHT;
        
        if (strcmp(measured->jedec_id, expected->jedec_id) == 0) {
            result.breakdown.jedec_id_score = 100.0;
            weighted_score += JEDEC_WEIGHT * 100.0;
        } else {
            result.breakdown.jedec_id_score = 0.0;
        }
    }
    
    // 2. Read Speed Deviation (20% weight)
    if (measured->read_speed_max > 0 && expected->read_speed_max > 0) {
        result.breakdown.read_speed_available = true;
        factors_available++;
        total_weight_available += READ_WEIGHT;
        
        float deviation = fabs(measured->read_speed_max - expected->read_speed_max) / expected->read_speed_max;
        deviation = fmax(0, deviation - MEASUREMENT_UNCERTAINTY);
        float normalized_score = fmax(0, 100.0 * (1.0 - deviation / READ_TOLERANCE));
        
        result.breakdown.read_speed_score = normalized_score;
        weighted_score += READ_WEIGHT * normalized_score;
    }
    
    // 3. Write Speed (Page Program) - COMPLETELY SKIPPED
    result.breakdown.write_speed_available = false;
    result.breakdown.write_speed_score = 0.0;
    
    // 4. Erase Speed Deviation (10% weight)
    if (measured->erase_speed > 0 && expected->erase_speed > 0) {
        result.breakdown.erase_speed_available = true;
        factors_available++;
        total_weight_available += ERASE_WEIGHT;
        
        float deviation = fabs(measured->erase_speed - expected->erase_speed) / expected->erase_speed;
        deviation = fmax(0, deviation - MEASUREMENT_UNCERTAINTY);
        float normalized_score = fmax(0, 100.0 * (1.0 - deviation / ERASE_TOLERANCE));
        
        result.breakdown.erase_speed_score = normalized_score;
        weighted_score += ERASE_WEIGHT * normalized_score;
    }
    
    // 5. Clock Speed Profile Match - COMPLETELY SKIPPED
    result.breakdown.clock_profile_available = false;
    result.breakdown.clock_profile_score = 0.0;
    
    result.factors_used = factors_available;
    
    // Handle insufficient data
    if (factors_available < 2) {
        snprintf(result.warning_message, sizeof(result.warning_message),
                 "WARNING_INSUFFICIENT_DATA: Only %d factors available", factors_available);
    }
    
    // Handle missing JEDEC ID (critical)
    if (!result.breakdown.jedec_id_available) {
        result.overall_confidence = 0.0;
        snprintf(result.warning_message, sizeof(result.warning_message),
                 "CRITICAL: JEDEC ID missing");
        return result;
    }
    
    // No redistribution - missing data = 0% for that factor
    result.overall_confidence = fmin(100.0, weighted_score);
    
    // Flag low-confidence components
    char low_conf_msg[256] = "";
    bool has_low_conf = false;
    
    if (result.breakdown.jedec_id_available && result.breakdown.jedec_id_score < 50.0) {
        strcat(low_conf_msg, "JEDEC ");
        has_low_conf = true;
    }
    
    if (result.breakdown.read_speed_available && result.breakdown.read_speed_score < 50.0) {
        strcat(low_conf_msg, "READ ");
        has_low_conf = true;
    }
    
    if (result.breakdown.erase_speed_available && result.breakdown.erase_speed_score < 50.0) {
        strcat(low_conf_msg, "ERASE ");
        has_low_conf = true;
    }
    
    if (has_low_conf) {
        char temp[256];
        snprintf(temp, sizeof(temp), "Low confidence factors: %s", low_conf_msg);
        strncpy(result.warning_message, temp, sizeof(result.warning_message) - 1);
    }
    
    return result;
}


// ============================================================================
// D7.2.2: chip_match_database()
// ============================================================================

match_status_t chip_match_database(FlashChipData* test_data) {
    if (database_entry_count == 0) {
        printf("[ERROR] ERROR_NO_DATABASE: No database loaded\n");
        return MATCH_UNKNOWN;
    }
    
    for (int i = 0; i < TOP_MATCHES_COUNT; i++) {
        match_results[i].confidence.overall_confidence = 0.0;
        match_results[i].status = MATCH_UNKNOWN;
        match_results[i].database_index = -1;
        match_results[i].has_outliers = false;
    }
    
    printf("\n====================================\n");
    printf(" Chip Matching Algorithm\n");
    printf(" Weights: JEDEC 40%%, Read 20%%, Erase 10%%\n");
    printf(" (Write/Page Program & Clock Speed SKIPPED)\n");
    printf("====================================\n\n");
    printf("Comparing against %d database entries...\n\n", database_entry_count);
    
    bool has_outlier = false;
    
    for (int i = 0; i < database_entry_count; i++) {
        confidence_result_t conf = chip_calculate_confidence(test_data, &database[i]);
        
        // Check for performance outliers
        if (test_data->read_speed_max > 0 && database[i].read_speed_max > 0) {
            float deviation = fabs(test_data->read_speed_max - database[i].read_speed_max) / 
                             database[i].read_speed_max;
            if (deviation > 0.50) {
                has_outlier = true;
                printf("[INFO] WARNING_PERFORMANCE_OUTLIER detected for %s (Read speed)\n",
                       database[i].chip_model);
            }
        }
        
        // Insert into top 3
        for (int j = 0; j < TOP_MATCHES_COUNT; j++) {
            if (conf.overall_confidence > match_results[j].confidence.overall_confidence) {
                for (int k = TOP_MATCHES_COUNT - 1; k > j; k--) {
                    match_results[k] = match_results[k - 1];
                }
                
                match_results[j].chip_data = database[i];
                match_results[j].confidence = conf;
                match_results[j].database_index = i;
                break;
            }
        }
    }
    
    match_results[0].has_outliers = has_outlier;
    if (has_outlier) {
        printf("\n[INFO] WARNING_PERFORMANCE_OUTLIER: Performance deviations >50%% detected\n");
    }
    
    // Determine match status
    if (match_results[0].confidence.overall_confidence >= 95.0 && 
        strcmp(test_data->jedec_id, match_results[0].chip_data.jedec_id) == 0) {
        match_results[0].status = MATCH_FOUND;
        printf("✓ FOUND: Exact match with %.1f%% confidence\n",
               match_results[0].confidence.overall_confidence);
    } else if (match_results[0].confidence.overall_confidence >= 70.0) {
        match_results[0].status = MATCH_BEST_MATCH;
        printf("~ BEST MATCH: Closest match with %.1f%% confidence\n",
               match_results[0].confidence.overall_confidence);
    } else {
        match_results[0].status = MATCH_UNKNOWN;
        printf("✗ UNKNOWN: No confident match found (best: %.1f%%)\n",
               match_results[0].confidence.overall_confidence);
    }
    
    // Display top 3 matches
    printf("\n--- Top %d Matches ---\n", TOP_MATCHES_COUNT);
    for (int i = 0; i < TOP_MATCHES_COUNT; i++) {
        if (match_results[i].database_index >= 0) {
            printf("%d. %s - %s (%.1f%% confidence)\n",
                   i + 1,
                   match_results[i].chip_data.company,
                   match_results[i].chip_data.chip_model,
                   match_results[i].confidence.overall_confidence);
            printf("   JEDEC: %s\n", match_results[i].chip_data.jedec_id);
            printf("   Factor breakdown:\n");
            
            if (match_results[i].confidence.breakdown.jedec_id_available)
                printf("    - JEDEC ID (40%%): %.0f%%\n", match_results[i].confidence.breakdown.jedec_id_score);
            if (match_results[i].confidence.breakdown.read_speed_available)
                printf("    - Read Speed (20%%): %.0f%%\n", match_results[i].confidence.breakdown.read_speed_score);
            if (match_results[i].confidence.breakdown.erase_speed_available)
                printf("    - Erase Speed (10%%): %.0f%%\n", match_results[i].confidence.breakdown.erase_speed_score);
            
            printf("\n");
        }
    }
    
    return match_results[0].status;
}
