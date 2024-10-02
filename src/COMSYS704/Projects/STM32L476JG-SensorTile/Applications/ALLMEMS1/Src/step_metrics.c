#include "step_metrics.h"

#define DYNAMIC_THRESHOLD_DEFAULT_VALUE 1000U

uint8_t looking_for_max = 1;

void metrics_buffer_init(MetricsType *metrics, uint16_t size)
{
	int32_t *buffer = (int32_t *)calloc(size, sizeof(int32_t));

	metrics->data = buffer;
	metrics->size = size;
	metrics->index = 0;
    metrics->center = size/2;
    metrics->step_detected = 0;
    metrics->dynamic_threshold = DYNAMIC_THRESHOLD_DEFAULT_VALUE;
}

void metrics_buffer_push(MetricsType *metrics, int32_t entry)
{
	metrics->data[metrics->index] = entry;

	metrics->index = (metrics->index + 1) % metrics->size;
    metrics->center = (metrics->center + 1) % metrics->size;

    // ---------------- Step Counting Algorithm ----------------
    uint8_t local_max_detected = 0;
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

    if (max_index == metrics->center)
    {
        local_max_detected = 1;
    }

    if(local_max_detected && max >= metrics->dynamic_threshold)
    {
        metrics->step_detected = 1;
    }
    else
    {
        metrics->step_detected = 0;
    }


    // ---------------------------------------------------------
    
}

void metrics_buffer_free(MetricsType *metrics)
{
	free(metrics->data);
}
