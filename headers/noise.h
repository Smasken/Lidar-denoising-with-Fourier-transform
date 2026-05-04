#ifndef NOISE_H
#define NOISE_H

#include "image.h"

void gaussian_noise(Image* img, float sigma);
void dropout_noise(Image* img, float prob);

#endif