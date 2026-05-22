#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "../headers/segmentation.h"

// Recreate point z from range image pixel coordinates.
// Uses the same elevation mapping as `normals.c`.
static int reconstruct_pz(int x, int y, int width, int height, float range, float* out_pz)
{
    if (range <= 0.0f) return 0;
    const float MIN_ELEV = -24.9f;
    const float MAX_ELEV = 2.0f;
    float step = (MAX_ELEV - MIN_ELEV) / (float)(height - 1);
    float elevation_deg = MIN_ELEV + (float)y * step;
    float elevation = elevation_deg * (float)M_PI / 180.0f;
    *out_pz = range * sinf(elevation);
    return 1;
}

int save_ground_overlay_as_ppm(Normal* normals, Image* range_img, int width, int height,
                                const char* filename, float nz_threshold, float h_threshold,
                                float min_range, float max_range)
{
    if (!range_img || !filename) return 0;

    FILE* f = fopen(filename, "wb");
    if (!f) return 0;

    float range_span = max_range - min_range;
    if (range_span == 0.0f) range_span = 1.0f;

    fprintf(f, "P6\n%d %d\n255\n", width, height);

    for (int i = 0; i < width * height; i++) {
        unsigned char rgb[3] = {0,0,0};

        int is_ground = 0;
        if (normals && normals[i].valid) {
            if (normals[i].nz >= nz_threshold) {
                // also enforce absolute height threshold
                int x = i % width;
                int y = i / width;
                float range = range_img->data[i];
                float pz = 0.0f;
                if (reconstruct_pz(x, y, width, height, range, &pz)) {
                    if (pz <= h_threshold) {
                        is_ground = 1;
                    }
                }
            }
        }

        if (is_ground) {
            rgb[0] = 255; // orange: 255,165,0
            rgb[1] = 165;
            rgb[2] = 0;
        } else {
            unsigned char pix = 0;
            // Prefer normals-based visualization (Z component -> brightness)
            if (normals && normals[i].valid) {
                float v = normals[i].nz;
                float val = (v * 0.5f) + 0.5f; // map [-1,1] -> [0,1]
                if (val < 0.0f) val = 0.0f;
                if (val > 1.0f) val = 1.0f;
                pix = (unsigned char)(val * 255.0f);
            } else {
                // fallback to range-based grayscale
                float valr = range_img->data[i];
                if (valr < 0.0f) {
                    pix = 0;
                } else {
                    float norm = (valr - min_range) / range_span;
                    norm = 1.0f - norm;
                    if (norm < 0.0f) norm = 0.0f;
                    if (norm > 1.0f) norm = 1.0f;
                    pix = (unsigned char)(norm * 255.0f);
                }
            }
            rgb[0] = pix; rgb[1] = pix; rgb[2] = pix;
        }

        fwrite(rgb, 1, 3, f);
    }

    fclose(f);
    return 1;
}
