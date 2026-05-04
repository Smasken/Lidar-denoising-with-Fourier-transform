#include <stdlib.h>
#include <math.h>
#include "noise.h"

// Box-Muller transform
static float rand_normal()
{
    float u1 = (rand() + 1.0f) / (RAND_MAX + 2.0f);
    float u2 = (rand() + 1.0f) / (RAND_MAX + 2.0f);

    return sqrtf(-2.0f * logf(u1)) * cosf(2.0f * M_PI * u2);
}

void gaussian_noise(Image* img, float sigma)
{
    int size = img->width * img->height;

    for (int i = 0; i < size; i++) {
        if (img->data[i] >= 0) { // ignore invalid pixels
            img->data[i] += sigma * rand_normal();
        }
    }
}

void dropout_noise(Image* img, float prob)
{
    int size = img->width * img->height;

    for (int i = 0; i < size; i++) {
        float r = (float)rand() / RAND_MAX;
        if (r < prob) {
            img->data[i] = -1.0f;
        }
    }
}