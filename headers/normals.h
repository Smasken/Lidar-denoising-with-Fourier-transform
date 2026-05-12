#ifndef NORMALS_H
#define NORMALS_H

#include "image.h"

typedef struct {
    float nx, ny, nz;
    int valid;
} Normal;

// Compute surface normals from a range image. Returns an array of
// width*height Normal structs (row-major). Caller must free.
Normal* compute_surface_normals(Image* range_img);

// Save normal map visualization as a PGM (grayscale) using the Z
// component (up = white, down = black). Invalid normals written as 0.
int save_normal_map_as_pgm(Normal* normals, int width, int height, const char* filename);

#endif
