#include "filter.h"

FilterType filter_init(uint16_t size)
{
	FilterType filter;

	int32_t *buffer = (int32_t *)malloc(size * sizeof(int32_t));

	filter.data = buffer;
	filter.size = size;
	filter.count = 0;
}

void filter_free(FilterType *filter)
{
}

void filter_push(FilterType *filter, int32_t entry)
{
	if (filter->count < filter->size)
	{
		filter->count += 1;
		filter->total += entry;
		filter->average = filter->total / filter->count;
	}
	else
	{
		filter->
	}
}