#ifndef LIDAR_LOADER_H
#define LIDAR_LOADER_H

typedef struct {
    float x;
    float y;
    float z;
    float intensity;
} Point;

typedef struct {
    Point* points;
    int num_points;
} LidarData;

LidarData* load_lidar_data(const char* filename);
void free_lidar_data(LidarData* data);

#endif