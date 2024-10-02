#include "step_metrics.h"
#include <stdlib.h>

// needed for XPRINTF
#include "SensorTile.h"
#include "SensorTile_bus.h"
#include "spi.h"
#include "ALLMEMS1_config.h"

#define DYNAMIC_THRESHOLD_DEFAULT_VALUE 1000U
#define TROUGH_SEARCH_TIMEOUT_MS 2000U
#define ACC_SAMPLING_FREQ_HZ 10U // should really import this constant from main

uint8_t looking_for_max = 1;

void metrics_buffer_init(MetricsType *metrics, uint16_t size)
{
	int32_t *buffer = (int32_t *)calloc(size, sizeof(int32_t));
//	int32_t *step_history = (int32_t *)calloc(1000 * sizeof(int32_t)); //memory for 1000 steps

	metrics->data = buffer;
	metrics->size = size;
	metrics->index = 0;
    metrics->center = size/2;
    metrics->step_detected = 0;
    metrics->dynamic_threshold = DYNAMIC_THRESHOLD_DEFAULT_VALUE;
//	metrics->step_history = step_history;
}

void metrics_buffer_push(MetricsType *metrics, int32_t entry)
{
	metrics->data[metrics->index] = entry;

	metrics->index = (metrics->index + 1) % metrics->size;
    metrics->center = (metrics->center + 1) % metrics->size;

    // ---------------- Step Counting Algorithm ----------------

    if(looking_for_max)
    {
		XPRINTF("looking for peak\t")

    	int32_t max = 0;
    	uint16_t max_index = 0;

    	for (uint16_t i = 0; i < metrics->size; ++i)
    	{
    	    if (metrics->data[i] > max)
    	    {
    	        max = metrics->data[i];
    	        max_index = i;
    	    }
    	}

    	if ((max_index == metrics->center) && (max >= metrics->dynamic_threshold))
		{
			XPRINTF("Peak detected\t")
			looking_for_max = 0;
			metrics->counter = 0;
		}

    }
    else
    {
		XPRINTF("looking for trough\t")

		int32_t min = 0x7FFFFFFF;
		uint16_t min_index = 0;

		for (uint16_t i = 0; i < metrics->size; ++i)
		{
			if (metrics->data[i] < min)
			{
				min = metrics->data[i];
				min_index = i;
			}
		}

		if ((min_index == metrics->center) && (min <= 500))
		{
			looking_for_max = 1;
			metrics->step_detected = 1;

			// keep record of step time 
			XPRINTF("Found trough \t")
		}

		// If no trough is detected, start looking for peak again
		if(metrics->counter > TROUGH_SEARCH_TIMEOUT_MS/ACC_SAMPLING_FREQ_HZ)  
		{
			looking_for_max = 1;
			XPRINTF("reset Peak\t")
		}

    }


    // ---------------------------------------------------------
    
}

void metrics_buffer_free(MetricsType *metrics)
{
	free(metrics->data);
}

// counter increments every time the accelerometer is sampled from main loop
void metrics_counter(MetricsType *metrics)
{
	metrics->counter++;
	XPRINTF("Curr metrics_count: %d \t", metrics->counter);
	
}
