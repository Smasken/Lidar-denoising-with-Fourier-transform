#ifndef DISCONTINUITY_H
#define DISCONTINUITY_H

#include "image.h"

// Compute depth discontinuities from a range image. Returns a new Image
// where pixels are 1.0f for discontinuity, 0.0f for no discontinuity,
// and -1.0f for invalid/missing input. Caller must free.
Image* compute_depth_discontinuities(Image* range_img, float threshold);

// Save discontinuity image as a binary PGM (edges white)
int save_discontinuity_as_pgm(Image* disc_img, const char* filename);

#endif
