#include <math.h>
#include "filters.h"

// Gaussian Low-Pass Filter

static void gaussian_filter(float complex* data, int width, int height, float sigma)
{
    int cx = width / 2;
    int cy = height / 2;

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {

            int dx = x - cx;
            int dy = y - cy;

            float dist2 = dx * dx + dy * dy;

            float H = expf(-dist2 / (2.0f * sigma * sigma));

            data[y * width + x] *= H;
        }
    }
}

/* =======================
   Dispatcher
   ======================= */
void apply_filter(float complex* data, int width, int height,
                  FilterType type, float param)
{
    switch (type) {

        case FILTER_NONE:
            // do nothing
            break;

        case FILTER_GAUSSIAN:
            gaussian_filter(data, width, height, param);
            break;

        default:
            break;
    }
}