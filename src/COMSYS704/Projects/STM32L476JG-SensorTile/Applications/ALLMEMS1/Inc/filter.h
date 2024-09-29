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
	int32_t average;
	uint16_t count;
	int32_t total;

} FilterType;

// ------------------------------------------------

// --------- Public Function Declarations ---------

FilterType filter_init(uint16_t size);
void filter_free(FilterType *filter);
void filter_push(FilterType *filter, int32_t entry);

// ------------------------------------------------

#endif	// FILTER_H
