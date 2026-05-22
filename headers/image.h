#ifndef IMAGE_H
#define IMAGE_H

#include "lidar.h"

typedef struct {
    float* data; // 1D array, row-major order
    int width;
    int height;
} Image;

Image* create_image(int width, int height);
void free_image(Image* img);
Image* lidar_to_range_image(LidarData* lidar, int width, int height);
int save_image_as_pgm(Image* img, const char* filename);
void image_range(Image* img, float* out_min, float* out_max);
void fill_holes(Image* img, int iterations);
void median_filter(Image* img);
Image* copy_image(Image* img);

#endif