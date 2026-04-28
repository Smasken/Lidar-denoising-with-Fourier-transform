#include <stdlib.h>
#include <math.h>
#include <complex.h>

#include "image.h"

/* =======================
   Bit reversal
   ======================= */
static void bit_reverse(float complex* data, int n)
{
    int j = 0;
    for (int i = 1; i < n; i++) {
        int bit = n >> 1;
        while (j & bit) {
            j ^= bit;
            bit >>= 1;
        }
        j |= bit;

        if (i < j) {
            float complex temp = data[i];
            data[i] = data[j];
            data[j] = temp;
        }
    }
}

/* =======================
   1D FFT
   ======================= */
void fft1d(float complex* data, int n)
{
    bit_reverse(data, n);

    for (int len = 2; len <= n; len <<= 1) {
        float angle = -2.0f * M_PI / len;
        
        float complex wlen = cosf(angle) + sinf(angle) * I;

        for (int i = 0; i < n; i += len) {
            float complex w = 1.0f;

            for (int j = 0; j < len / 2; j++) {
                float complex u = data[i + j];
                float complex v = data[i + j + len/2] * w;

                data[i + j] = u + v;
                data[i + j + len/2] = u - v;

                w *= wlen;
            }
        }
    }
}

/* =======================
   1D IFFT
   ======================= */
void ifft1d(float complex* data, int n)
{
    for (int i = 0; i < n; i++)
        data[i] = conjf(data[i]);

    fft1d(data, n);

    for (int i = 0; i < n; i++)
        data[i] = conjf(data[i]) / n;
}

/* =======================
   2D FFT
   ======================= */
void fft2d(float complex* data, int width, int height)
{
    // FFT rows
    for (int y = 0; y < height; y++) {
        fft1d(&data[y * width], width);
    }

    // FFT columns
    float complex* column = malloc(sizeof(float complex) * height);

    for (int x = 0; x < width; x++) {

        // extract column
        for (int y = 0; y < height; y++) {
            column[y] = data[y * width + x];
        }

        fft1d(column, height);

        // write back
        for (int y = 0; y < height; y++) {
            data[y * width + x] = column[y];
        }
    }

    free(column);
}

// 2D IFFT

void ifft2d(float complex* data, int width, int height)
{
    // IFFT rows
    for (int y = 0; y < height; y++) {
        ifft1d(&data[y * width], width);
    }

    // IFFT columns
    float complex* column = malloc(sizeof(float complex) * height);

    for (int x = 0; x < width; x++) {

        for (int y = 0; y < height; y++) {
            column[y] = data[y * width + x];
        }

        ifft1d(column, height);

        for (int y = 0; y < height; y++) {
            data[y * width + x] = column[y];
        }
    }

    free(column);
}

void fft_pipeline(Image* img)
{
    int w = img->width;
    int h = img->height;
    int size = w * h;

    float complex* data = malloc(sizeof(float complex) * size);
    if (!data) return;

    // Copy image to complex form
    for (int i = 0; i < size; i++) {
        data[i] = img->data[i];
    }

    // Forward FFT
    fft2d(data, w, h);

    // Inverse FFT
    ifft2d(data, w, h);

    // Copy back (real part)
    for (int i = 0; i < size; i++) {
        img->data[i] = crealf(data[i]);
    }

    free(data);
}