#include <stdio.h>
#include "lidar.h"
#include "image.h"
#include "diffusion.h"
#include "fft.h"

int main()
{
    LidarData* scan = load_lidar_data("data/input_images (KITTI 2011_09_26_drive_0060)/0000000000.bin");

    if (!scan) {
        printf("Failed to read LIDAR data\n");
        return 1;
    }

    printf("loaded %d points\n", scan->num_points);

    /*
    for (int i = 0; i < 5; i++) {
        printf("Point %d: %.2f %.2f %.2f %.2f\n",
            i,
            scan->points[i].x,
            scan->points[i].y,
            scan->points[i].z,
            scan->points[i].intensity);
    }*/

    // Convert to range image
    Image* range_img = lidar_to_range_image(scan, 1024, 64);
    if (range_img) {
        printf("Created range image: %d x %d\n", range_img->width, range_img->height);

        fill_holes(range_img, 5); // fill empty pixels using neighbor averaging
        //median_filter(range_img); 
        //apply_diffusion(range_img, 10, 0.1f);

        save_image_as_pgm(range_img, "before_fft.pgm");
        fft_pipeline(range_img);
        save_image_as_pgm(range_img, "after_fft.pgm");

        free_image(range_img);
    }

    free_lidar_data(scan);
    return 0;
}