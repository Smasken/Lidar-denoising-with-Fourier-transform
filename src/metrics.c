#include <math.h>
#include "metrics.h"

float compute_mse(Image* img1, Image* img2)
{
    int size = img1->width * img1->height;
    float mse = 0.0f;
    int count = 0;

    for (int i = 0; i < size; i++) {
        float a = img1->data[i];
        float b = img2->data[i];

        if (a >= 0 && b >= 0) {
            float diff = a - b;
            mse += diff * diff;
            count++;
        }
    }

    return (count > 0) ? mse / count : 0.0f;
}

// Note the PSNR depends on a sensible max value for the data. 
// For LiDAR range images, 100 meters is a reasonable choice, adjust as necessary.
float compute_psnr(Image* ref, Image* img)
{
    float mse = compute_mse(ref, img);

    if (mse <= 1e-10f) {
        return 100.0f;
    }

    float max_val = 100.0f; // Adjust this based on the expected range of your data

    return 10.0f * log10f((max_val * max_val) / mse);
}