#include "filter.h"

void filter_init(FilterType *filter, uint16_t size)
{
	int32_t *buffer = (int32_t *)calloc(size, sizeof(int32_t));

	filter->data = buffer;
	filter->size = size;
	filter->count = 0;
	filter->total = 0;
	filter->index = 0;
	filter->average = 0;
}

void filter_push(FilterType *filter, int32_t entry)
{
	if(filter->count < filter->size)
		filter->count++;

	// Push entry into the total sum and subtract the oldest entry from the sum
	// Before the buffer is full, this just adds the entry and subtracts zero.
	filter->total += entry - filter->data[filter->index];

	// Register the entry into the next available index
	filter->data[filter->index] = entry;

	// Increment the index and overflow back to zero
	filter->index = (filter->index + 1) % filter->size;

	// Calculate the average based on the sum of entries and number of entries
	filter->average = filter->total / filter->count;
}

void filter_free(FilterType *filter)
{
	free(filter->data);
}

void filter_flush(FilterType *filter)
{
	for (int i = 0; i < filter->size; ++i)
	{
		filter->data[i] = 0;
	}

	filter->count = 0;
	filter->total = 0;
	filter->index = 0;
	filter->average = 0;
	
}
