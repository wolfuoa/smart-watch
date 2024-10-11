#include "step_metrics.h"
#include <stdlib.h>

// Includes for XPRINTF
#include "SensorTile.h"
#include "SensorTile_bus.h"
#include "spi.h"
#include "ALLMEMS1_config.h"

// Constants used in step detection algorithm
#define DYNAMIC_THRESHOLD_DEFAULT_VALUE_HIGH 1000
#define DYNAMIC_THRESHOLD_DEFAULT_VALUE_LOW 500
#define AVERAGE_FREQUENCY_DEFAULT_COUNTS 100
#define TROUGH_SEARCH_TIMEOUT_MS 2000
#define MIN_MAX_OFFSET 500
#define ACC_SAMPLING_FREQ_HZ 10
#define PEAK_TO_PEAK_THRESHOLD 200
#define PEAK_DETECTION_COOLDOWN_MS 100
#define COOLDOWN_COUNTS 0

// Step detection global variables
uint8_t looking_for_max = 1;
int32_t min;
int32_t max;


// Filter and metrics initialization 
void metrics_buffer_init(MetricsType *metrics, uint16_t size)
{
	int32_t *buffer = (int32_t *)calloc(size, sizeof(int32_t));
	FilterType *high_threshold_filter = (FilterType*)malloc(sizeof(FilterType));
	FilterType *low_threshold_filter = (FilterType*)malloc(sizeof(FilterType));
	FilterType *frequency_filter = (FilterType*)malloc(sizeof(FilterType));

	filter_init(high_threshold_filter, 4);
	filter_init(low_threshold_filter, 4);
	filter_init(frequency_filter, 4);

	high_threshold_filter->average = DYNAMIC_THRESHOLD_DEFAULT_VALUE_HIGH;
	low_threshold_filter->average = DYNAMIC_THRESHOLD_DEFAULT_VALUE_LOW;
	frequency_filter->average = AVERAGE_FREQUENCY_DEFAULT_COUNTS;

	metrics->data = buffer;
	metrics->size = size;
	metrics->index = 0;
    metrics->center = size/2;
    metrics->step_detected = 0;
	metrics->high_threshold_filter = high_threshold_filter;
	metrics->low_threshold_filter = low_threshold_filter;
	metrics->frequency_filter = frequency_filter;
	metrics->debug = 0;
}

// Main step detection algoritm
void metrics_buffer_push(MetricsType *metrics, int32_t entry)
{
	metrics->data[metrics->index] = entry;
	metrics->index = (metrics->index + 1) % metrics->size;
    metrics->center = (metrics->center + 1) % metrics->size;


    // ---------------- Step Counting Algorithm ----------------

	// Start looking for peak
    if(looking_for_max)
    {
		metrics->debug = 0;

    	max = 0;
    	uint16_t max_index = metrics->index;

		// Update max value on each loop
    	for (uint16_t i = 0; i < metrics->size; ++i)
    	{
    	    if (metrics->data[i] > max)
    	    {
    	        max = metrics->data[i];
    	        max_index = i;
    	    }
    	}

		// Max value detected in center of the sliding window
    	if ((max_index == metrics->center))
		{
			XPRINTF("Peak detected\t")
			filter_push(metrics->high_threshold_filter, max);
			
			// Start looking for an associated trough if the max peak is above thresholds
			if ((max >= metrics->high_threshold_filter->average - MIN_MAX_OFFSET) && (max >= 500))
			{
				looking_for_max = 0;
				metrics->counter = 0;
			}
		}
    }
    else
    {
		metrics->debug = 1;
		int temp = (max - min);

		min = 0x7FFFFFFF;
		uint16_t min_index = metrics->index;

		// Update min value on each loop
		for (uint16_t i = 0; i < metrics->size; ++i)
		{
			if (metrics->data[i] < min)
			{
				min = metrics->data[i];
				min_index = i;
			}
		}

		// Min value detected in center of the sliding window
		if (min_index == metrics->center)
		{
			XPRINTF("Found trough \t")
			filter_push(metrics->low_threshold_filter, min);

			// Trough associated with peak detected, increment step, and push count value into frequency filter to find step frequency
			if((min <= metrics->low_threshold_filter->average + MIN_MAX_OFFSET) && ((max - min) > PEAK_TO_PEAK_THRESHOLD))
			{
				XPRINTF("STEP DETECTED \t")
				looking_for_max = 1;
				metrics->step_detected = 1;
				filter_push(metrics->frequency_filter, metrics->counter);
			}
		}

		// If no trough is detected after a timout period, start looking for peak again
		if(metrics->counter > TROUGH_SEARCH_TIMEOUT_MS/ACC_SAMPLING_FREQ_HZ)  
		{
			looking_for_max = 1;
			XPRINTF("reset Peak\t")

			// Reset dynamic thresholds to default and clear step frequency  
			filter_flush(metrics->high_threshold_filter);
			filter_flush(metrics->low_threshold_filter);
			filter_flush(metrics->frequency_filter);

			metrics->high_threshold_filter->average = DYNAMIC_THRESHOLD_DEFAULT_VALUE_HIGH;
			metrics->low_threshold_filter->average = DYNAMIC_THRESHOLD_DEFAULT_VALUE_LOW;
			metrics->frequency_filter->average = AVERAGE_FREQUENCY_DEFAULT_COUNTS;
		}
    }

    // ---------------------------------------------------------
}

// Clears buffers
void metrics_buffer_free(MetricsType *metrics)
{
	free(metrics->data);
}

// Counter increments every time the accelerometer is sampled from main loop
void metrics_counter(MetricsType *metrics)
{
	metrics->counter++;
}
