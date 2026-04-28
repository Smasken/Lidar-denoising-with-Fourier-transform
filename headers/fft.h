#ifndef FFT_H
#define FFT_H

#include "image.h"
#include <complex.h>

void fft1d(float complex* data, int n);
void ifft1d(float complex* data, int n);

void fft2d(float complex* data, int width, int height);
void ifft2d(float complex* data, int width, int height);

void fft_pipeline(Image* img);

#endif