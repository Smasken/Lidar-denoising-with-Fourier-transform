#ifndef METRICS_H
#define METRICS_H

#include "image.h"

float compute_mse(Image* img1, Image* img2);
float compute_psnr(Image* ref, Image* img);

#endif