#include "step_metrics.h"

void metrics_buffer_init(MetricsType *metrics, uint16_t size)
{
	int32_t *buffer = (int32_t *)calloc(size, sizeof(int32_t));

	metrics->data = buffer;
	metrics->size = size;
	metrics->index = 0;
    metrics->peak_met_threshold = 0;
    metrics->peak_is_local_max = 0;
    metrics->peak_is_sloped = 0;
}


void metrics_buffer_push(MetricsType *metrics, int32_t entry)
{
	metrics->data[metrics->index] = entry;

	metrics->index = (metrics->index + 1) % metrics->size;

    uint32_t sum = 0;

    for(uint16_t i = 0; i < metrics->size/2; ++i)
    {
        sum += 0;
    }
}

void metrics_buffer_free(MetricsType *metrics)
{
	free(metrics->data);
}
