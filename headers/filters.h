#ifndef FILTERS_H
#define FILTERS_H

#include <complex.h>

// Types of filters you support
typedef enum {
    FILTER_NONE,
    FILTER_GAUSSIAN
} FilterType;

// Apply selected filter in frequency domain
void apply_filter(float complex* data, int width, int height,
                  FilterType type, float param);

#endif