#include <stdio.h>
#include <time.h>
#include "lidar.h"
#include "image.h"
#include "diffusion.h"
#include "fft.h"
#include "metrics.h"
#include "noise.h"

int main()
{
    LidarData* scan = load_lidar_data("data/input_images (KITTI 2011_09_26_drive_0060)/0000000000.bin");

    if (!scan) {
        printf("Failed to read LIDAR data\n");
        return 1;
    }

    printf("loaded %d points\n", scan->num_points);

    // Convert to range image
    Image* range_img = lidar_to_range_image(scan, 1024, 64);
    if (range_img) {
        printf("Created range image: %d x %d\n", range_img->width, range_img->height);

        fill_holes(range_img, 5); // fill empty pixels using neighbor averaging
        //median_filter(range_img); 
        //apply_diffusion(range_img, 10, 0.1f);

        save_image_as_pgm(range_img, "before_fft.pgm");

        /*
        fft_pipeline function applies transform, filters, and inverse transform. 
        Select filter and parameter (sigma, as a float). 
        Filter choices: 
        FILTER_NONE, FILTER_GAUSSIAN
        */
        
        // Create clean reference
    Image* original = copy_image(range_img);

    // Create noisy version
    Image* noisy = copy_image(range_img);
    gaussian_noise(noisy, 0.5f);
    save_image_as_pgm(noisy, "noisy.pgm");

    // Create copies for each method
    Image* img_diff = copy_image(noisy);
    Image* img_fft  = copy_image(noisy);

    printf("\n--- Running experiment ---\n");

    // ====================
    // Diffusion
    // ====================
    clock_t start = clock();

    apply_diffusion(img_diff, 20, 0.1f);

    clock_t end = clock();
    float time_diff = 1000.0f * (end - start) / CLOCKS_PER_SEC;

    float mse_diff  = compute_mse(original, img_diff);
    float psnr_diff = compute_psnr(original, img_diff);

    // ====================
    // FFT
    // ====================
    start = clock();

    fft_pipeline(img_fft, FILTER_GAUSSIAN, 30.0f);

    end = clock();
    float time_fft = 1000.0f * (end - start) / CLOCKS_PER_SEC;

    float mse_fft  = compute_mse(original, img_fft);
    float psnr_fft = compute_psnr(original, img_fft);

    // ====================
    // Print results
    // ====================
    printf("\nMethod        MSE        PSNR       Time(ms)\n");
    printf("------------------------------------------------\n");
    printf("Diffusion   %8.4f   %8.2f   %8.2f\n", mse_diff, psnr_diff, time_diff);
    printf("FFT         %8.4f   %8.2f   %8.2f\n", mse_fft, psnr_fft, time_fft);

    // Optional: save outputs
    save_image_as_pgm(img_diff, "diffusion.pgm");
    save_image_as_pgm(img_fft,  "fft.pgm");

    // Cleanup
    free_image(original);
    free_image(noisy);
    free_image(img_diff);
    free_image(img_fft);

    free_lidar_data(scan);
    return 0;
    }
}