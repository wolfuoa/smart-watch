/**
 ******************************************************************************
 * @file    step_metrics.h
 * @author  nwol626
 * @version 1.0.0
 * @date    1-10-2024
 * @brief   Metrics initialization and utility functions
 ******************************************************************************
 */

#ifndef STEP_METRICS_H
#define STEP_METRICS_H

#include <stdint.h>

// ---------------- Public Typedef ----------------

typedef struct MetricsType_t
{
	uint16_t size;
	int32_t *data;
	uint16_t index;
    uint8_t peak_met_threshold;
    uint8_t peak_is_local_max;
    uint8_t peak_is_sloped;

} MetricsType;

// ------------------------------------------------

// --------- Public Function Declarations ---------

void metrics_buffer_init(MetricsType *filter, uint16_t size);
void metrics_buffer_free(MetricsType *filter);
void metrics_buffer_push(MetricsType *filter, int32_t entry);

// ------------------------------------------------

#endif	// STEP_METRICS_H
