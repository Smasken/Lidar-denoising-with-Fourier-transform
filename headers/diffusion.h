#ifndef DIFFUSION_H
#define DIFFUSION_H

#include "image.h"

// Apply multiple diffusion iterations
void apply_diffusion(Image* img, int iterations, float alpha);

#endif