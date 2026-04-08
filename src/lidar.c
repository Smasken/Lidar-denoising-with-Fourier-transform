#include <stdio.h>
#include <stdlib.h>
#include "../headers/lidar.h"

LidarData* load_lidar_data(const char* filename) 
{
    FILE* file = fopen(filename, "rb"); //Read binary
    if (!file) {
        perror("Failed to open file"); 
        return NULL;
    }

    // Get file size and allocate memory for points
    fseek(file, 0, SEEK_END);
    long file_size = ftell(file);
    rewind(file);
    int num_points = file_size / (4 * sizeof(float)); // Each point has 4 floats (x, y, z, intensity)

    LidarData *points = malloc(num_points * sizeof(Point));

    if (!points) {
        perror("Memory allocation failed");
        fclose(file);
        return NULL;
    }

    size_t read = fread(points, sizeof(LidarData), num_points, file);
    if (read != num_points) {
        perror("Failed to read all data");
        free(points);
        fclose(file);
        return NULL;
    }

    fclose(file);

    LidarData *scan = malloc(sizeof(LidarData));
    scan->points = points;
    scan->num_points = num_points;
    return scan;
}

void free_lidar_data(LidarData* scan) 
{
    if (!scan) return ;
    free(scan->points);
    free(scan);
}