#include <stdlib.h>
#include <string.h>
#include "../headers/image.h"

void diffusion_step(Image* img, float alpha)
{
    int w = img->width;
    int h = img->height;

    float* temp = malloc(sizeof(float) * w * h);
    if (!temp) return;

    // Copy edges unchanged (simplest approach)
    memcpy(temp, img->data, sizeof(float) * w * h);

    for (int y = 1; y < h - 1; y++) {
        for (int x = 1; x < w - 1; x++) {

            int idx = y * w + x;

            float center = img->data[idx];
            float up     = img->data[(y - 1) * w + x];
            float down   = img->data[(y + 1) * w + x];
            float left   = img->data[y * w + (x - 1)];
            float right  = img->data[y * w + (x + 1)];

            float laplacian = up + down + left + right - 4.0f * center;

            temp[idx] = center + alpha * laplacian;
        }
    }

    memcpy(img->data, temp, sizeof(float) * w * h);
    free(temp);
}

void apply_diffusion(Image* img, int iterations, float alpha)
{
    for (int i = 0; i < iterations; i++) {
        diffusion_step(img, alpha);
    }
}