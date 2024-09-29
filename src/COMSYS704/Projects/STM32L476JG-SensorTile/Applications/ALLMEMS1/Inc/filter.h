/**
 ******************************************************************************
 * @file    filter.h
 * @author  nwol626
 * @version 1.0.0
 * @date    20-09-2024
 * @brief   Filter initialization and utility functions
 ******************************************************************************
 */

#ifndef FILTER_H
#define FILTER_H

#include <stdint.h>

// ---------------- Public Typedef ----------------

typedef struct FilterType_t
{
	uint16_t size;
	int32_t *data;
	int32_t mask;
	int32_t average;
	int32_t total;
	uint16_t index;
	uint16_t count;

} FilterType;

// ------------------------------------------------

// --------- Public Function Declarations ---------

void filter_init(FilterType *filter, uint16_t size);
void filter_free(FilterType *filter);
void filter_push(FilterType *filter, int32_t entry);

// ------------------------------------------------

#endif	// FILTER_H
