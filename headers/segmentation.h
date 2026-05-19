#ifndef SEGMENTATION_H
#define SEGMENTATION_H

#include "normals.h"
#include "image.h"

// Save an RGB overlay image where ground pixels (normals pointing up)
// are colored orange and other pixels retain the grayscale range visualization.
// Writes a binary PPM (P6). Returns 1 on success.
int save_ground_overlay_as_ppm(Normal* normals, Image* range_img, int width, int height, const char* filename, float nz_threshold);

#endif
