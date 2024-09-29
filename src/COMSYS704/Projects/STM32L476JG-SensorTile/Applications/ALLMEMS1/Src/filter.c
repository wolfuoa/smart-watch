#include "filter.h"

void filter_init(FilterType *filter, uint16_t size)
{
	int32_t *buffer = (int32_t *)calloc(0, size * sizeof(int32_t));

	filter->data = buffer;
	filter->size = size;
	filter->count = 0;
	filter->total = 0;
	filter->index = 0;
	filter->average = 0;
	filter->mask = size - 1;
}


void filter_push(FilterType *filter, int32_t entry)
{
	if(filter->count < filter->size)
		filter->count++;

	filter->total += entry - filter.data[filter.index];

	filter.data[filter.index] = entry;

	filter->index = (filter->index + 1) & filter.mask;

	filter->average = filter->total / filter->count;
}

void filter_free(FilterType *filter)
{
	free(filter->data);
}