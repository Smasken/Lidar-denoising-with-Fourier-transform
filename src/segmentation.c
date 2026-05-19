#include <stdio.h>
#include <stdlib.h>
#include <float.h>
#include "../headers/segmentation.h"

int save_ground_overlay_as_ppm(Normal* normals, Image* range_img, int width, int height, const char* filename, float nz_threshold)
{
    if (!range_img || !filename) return 0;

    FILE* f = fopen(filename, "wb");
    if (!f) return 0;

    // Determine range min/max as in save_image_as_pgm
    float min_range = FLT_MAX;
    float max_range = -FLT_MAX;
    for (int i = 0; i < width * height; i++) {
        float val = range_img->data[i];
        if (val >= 0) {
            if (val < min_range) min_range = val;
            if (val > max_range) max_range = val;
        }
    }
    float range_span = max_range - min_range;
    if (range_span == 0.0f) range_span = 1.0f;

    fprintf(f, "P6\n%d %d\n255\n", width, height);

    for (int i = 0; i < width * height; i++) {
        unsigned char rgb[3] = {0,0,0};

        int is_ground = 0;
        if (normals && normals[i].valid) {
            if (normals[i].nz >= nz_threshold) is_ground = 1;
        }

        if (is_ground) {
            rgb[0] = 255; // orange: 255,165,0
            rgb[1] = 165;
            rgb[2] = 0;
        } else {
            float val = range_img->data[i];
            unsigned char pix = 0;
            if (val < 0.0f) {
                pix = 0;
            } else {
                float norm = (val - min_range) / range_span;
                norm = 1.0f - norm;
                if (norm < 0.0f) norm = 0.0f;
                if (norm > 1.0f) norm = 1.0f;
                pix = (unsigned char)(norm * 255.0f);
            }
            rgb[0] = pix; rgb[1] = pix; rgb[2] = pix;
        }

        fwrite(rgb, 1, 3, f);
    }

    fclose(f);
    return 1;
}
