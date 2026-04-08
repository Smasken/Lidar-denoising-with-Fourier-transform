#include <stdio.h>
#include "../headers/lidar.h"

int main()
{
    LidarData* scan = load_lidar_data("/Users/danfriedman/Documents/Programming Project in Computational Science/fourier_project/data/input_images (KITTI 2011_09_26_drive_0060)/0000000000.bin");

    if (!scan) {
        printf("Failed to read LIDAR data\n");
        return 1;
    }

    printf("loaded %d points\n", scan->num_points);

    for (int i = 0; i < 5; i++) {
        printf("Point %d: %.2f %.2f %.2f %.2f\n",
            i,
            scan->points[i].x,
            scan->points[i].y,
            scan->points[i].z,
            scan->points[i].intensity);
    }

    free_lidar_data(scan);
    return 0;
}