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
    uint16_t center;
    uint8_t step_detected;
    int32_t dynamic_threshold;

} MetricsType;

// ------------------------------------------------

// --------- Public Function Declarations ---------

void metrics_buffer_init(MetricsType *metrics, uint16_t size);
void metrics_buffer_free(MetricsType *metrics);
void metrics_buffer_push(MetricsType *metrics, int32_t entry);

// ------------------------------------------------

#endif	// STEP_METRICS_H
