#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <float.h>
#include "../headers/image.h"
#include "../headers/lidar.h"

Image* create_image(int width, int height) {
    Image* img = malloc(sizeof(Image));
    if (!img) return NULL;
    img->data = calloc(width * height, sizeof(float));
    if (!img->data) {
        free(img);
        return NULL;
    }
    img->width = width;
    img->height = height;
    return img;
}

void free_image(Image* img) {
    if (img) {
        free(img->data);
        free(img);
    }
}

Image* lidar_to_range_image(LidarData* lidar, int width, int height) {
    if (!lidar || width <= 0 || height <= 0) return NULL;

    Image* img = create_image(width, height);
    if (!img) return NULL;

    // Initialize to -1 (no data)
    for (int i = 0; i < width * height; i++) {
        img->data[i] = -1.0f;
    }

    // For KITTI Velodyne HDL-64E, approximate elevation range
    float min_elev = -24.9f;
    float max_elev = 2.0f;

    // Approximate beam elevations (HDL-64E has 64 beams roughly equally spaced)
    float beam_elev[64];
    float step = (max_elev - min_elev) / 63.0f;
    for (int i = 0; i < 64; i++) {
        beam_elev[i] = min_elev + i * step;
    }

    for (int i = 0; i < lidar->num_points; i++) {
        Point p = lidar->points[i];
        float range = sqrtf(p.x * p.x + p.y * p.y + p.z * p.z);
        if (range <= 0.0f) continue;

        // Compute azimuth in degrees [0, 360)
        float azimuth = atan2f(p.y, p.x) * 180.0f / M_PI;
        if (azimuth < 0) azimuth += 360.0f;

        // Compute elevation in degrees
        float elevation = atan2f(p.z, sqrtf(p.x * p.x + p.y * p.y)) * 180.0f / M_PI;

        // Map to pixel coordinates
        int x = (int)(azimuth / 360.0f * width) % width;

        // Find the closest beam elevation
        int y = 0;
        float min_diff = fabsf(elevation - beam_elev[0]);
        for (int j = 1; j < height; j++) {
            float diff = fabsf(elevation - beam_elev[j]);
            if (diff < min_diff) {
                min_diff = diff;
                y = j;
            }
        }

        // Take the minimum range for each bin
        int idx = y * width + x;
        if (img->data[idx] < 0.0f || range < img->data[idx]) {
            img->data[idx] = range;
        }
    }

    return img;
}

int save_image_as_pgm(Image* img, const char* filename) {
    if (!img || !filename) return 0;

    FILE* file = fopen(filename, "wb");
    if (!file) return 0;

    /* Find min and max range for normalization (alternative instead of fixed values)
    float min_range = FLT_MAX;
    float max_range = -FLT_MAX;
    for (int i = 0; i < img->width * img->height; i++) {
        float val = img->data[i];
        if (val >= 0) {
            if (val < min_range) min_range = val;
            if (val > max_range) max_range = val;
        }
    }
    

    if (max_range <= min_range) {
        // All same or no data
        max_range = min_range + 1.0f;
    }
    */

    float min_range = 1.0f;
    float max_range = 80.0f;
    float range_span = max_range - min_range;

    // Write PGM header
    fprintf(file, "P5\n%d %d\n255\n", img->width, img->height);

    for (int i = 0; i < img->width * img->height; i++) {
        float val = img->data[i];
        unsigned char pixel;

        if (val < 0.0f) {
            pixel = 0;
        } else {
            float norm = (val - min_range) / range_span;
            norm = 1.0f - norm;

            if (norm < 0.0f) norm = 0.0f;
            if (norm > 1.0f) norm = 1.0f;

            pixel = (unsigned char)(norm * 255.0f);
        }

        fwrite(&pixel, 1, 1, file);
    }

    fclose(file);
    return 1;
}

void fill_holes(Image* img, int iterations)
{
    int w = img->width;
    int h = img->height;

    float* temp = malloc(sizeof(float) * w * h);
    if (!temp) return;

    for (int it = 0; it < iterations; it++) {

        memcpy(temp, img->data, sizeof(float) * w * h);

        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++) {

                int idx = y * w + x;

                // Only fill missing pixels
                if (img->data[idx] >= 0.0f)
                    continue;

                float sum = 0.0f;
                int count = 0;

                // 4-neighborhood (up, down, left, right)
                int dx[4] = {1, -1, 0, 0};
                int dy[4] = {0, 0, 1, -1};

                for (int k = 0; k < 4; k++) {
                    int nx = x + dx[k];
                    int ny = y + dy[k];

                    if (nx >= 0 && nx < w && ny >= 0 && ny < h) {
                        float val = img->data[ny * w + nx];
                        if (val >= 0.0f) {
                            sum += val;
                            count++;
                        }
                    }
                }

                if (count > 0) {
                    temp[idx] = sum / count;
                }
            }
        }

        // Copy back
        memcpy(img->data, temp, sizeof(float) * w * h);
    }

    free(temp);
}