#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <omp.h>
#include "../headers/discontinuity.h"

Image* compute_depth_discontinuities(Image* range_img, float threshold)
{
    if (!range_img) return NULL;
    int w = range_img->width;
    int h = range_img->height;

    Image* out = create_image(w, h);
    if (!out) return NULL;

    // Initialize to -1 for invalid
    for (int i = 0; i < w * h; i++) out->data[i] = -1.0f;

    #pragma omp parallel for collapse(2) schedule(static)
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            int idx = y * w + x;
            float center = range_img->data[idx];
            if (center <= 0.0f) { out->data[idx] = -1.0f; continue; }

            float max_diff = 0.0f;

            // 4-neighborhood
            int dx[4] = {1, -1, 0, 0};
            int dy[4] = {0, 0, 1, -1};
            for (int k = 0; k < 4; k++) {
                int nx = x + dx[k];
                int ny = y + dy[k];
                if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
                float nval = range_img->data[ny * w + nx];
                if (nval <= 0.0f) continue;
                float diff = fabsf(nval - center);
                if (diff > max_diff) max_diff = diff;
            }

            out->data[idx] = (max_diff >= threshold) ? 1.0f : 0.0f;
        }
    }

    return out;
}

int save_discontinuity_as_pgm(Image* disc_img, const char* filename)
{
    if (!disc_img || !filename) return 0;
    FILE* f = fopen(filename, "wb");
    if (!f) return 0;
    int w = disc_img->width;
    int h = disc_img->height;
    fprintf(f, "P5\n%d %d\n255\n", w, h);
    for (int i = 0; i < w * h; i++) {
        unsigned char pix = 0;
        float v = disc_img->data[i];
        if (v > 0.5f) pix = 255;
        else pix = 0;
        fwrite(&pix, 1, 1, f);
    }
    fclose(f);
    return 1;
}
