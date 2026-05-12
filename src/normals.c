#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include "../headers/normals.h"

// Assumed elevation range used in lidar_to_range_image
static const float MIN_ELEV = -24.9f;
static const float MAX_ELEV = 2.0f;

// Reconstruct 3D point from range image pixel (row y, col x)
// Returns 0 if invalid range, 1 otherwise. Out arguments set when valid.
static int reconstruct_point(int x, int y, int width, int height, float range,
                             float* px, float* py, float* pz)
{
    if (range <= 0.0f) return 0;

    float azimuth_deg = ( (float)x / (float)width ) * 360.0f;
    float azimuth = azimuth_deg * (float)M_PI / 180.0f;

    float step = (MAX_ELEV - MIN_ELEV) / (float)(height - 1);
    float elevation_deg = MIN_ELEV + (float)y * step;
    float elevation = elevation_deg * (float)M_PI / 180.0f;

    float cos_e = cosf(elevation);
    *px = range * cos_e * cosf(azimuth);
    *py = range * cos_e * sinf(azimuth);
    *pz = range * sinf(elevation);
    return 1;
}

// Try to find a valid neighbor along x (right) up to max_offset steps.
static int find_neighbor_right(Image* img, int x, int y, int max_offset, int* out_x)
{
    int w = img->width;
    for (int off = 1; off <= max_offset; off++) {
        int nx = x + off;
        if (nx >= w) break;
        if (img->data[y * w + nx] > 0.0f) { *out_x = nx; return 1; }
    }
    return 0;
}

// Try to find a valid neighbor along y (down) up to max_offset steps.
static int find_neighbor_down(Image* img, int x, int y, int max_offset, int* out_y)
{
    int w = img->width;
    int h = img->height;
    for (int off = 1; off <= max_offset; off++) {
        int ny = y + off;
        if (ny >= h) break;
        if (img->data[ny * w + x] > 0.0f) { *out_y = ny; return 1; }
    }
    return 0;
}

Normal* compute_surface_normals(Image* range_img)
{
    if (!range_img) return NULL;

    int w = range_img->width;
    int h = range_img->height;
    Normal* normals = malloc(sizeof(Normal) * w * h);
    if (!normals) return NULL;

    // For robustness, allow searching up to 2 pixels for valid neighbors
    const int MAX_OFFSET = 2;

    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            int idx = y * w + x;
            normals[idx].valid = 0;
            float r = range_img->data[idx];
            if (r <= 0.0f) continue;

            // Find right neighbor
            int rx = x + 1;
            int ry = y + 1;
            int found_r = 0;
            int found_b = 0;

            if (rx < w && range_img->data[y * w + rx] > 0.0f) {
                found_r = 1;
            } else {
                // search a little further to the right
                found_r = find_neighbor_right(range_img, x, y, MAX_OFFSET, &rx);
            }

            if (ry < h && range_img->data[ry * w + x] > 0.0f) {
                found_b = 1;
            } else {
                found_b = find_neighbor_down(range_img, x, y, MAX_OFFSET, &ry);
            }

            if (!found_r || !found_b) {
                // If we don't have both neighbors, mark invalid
                normals[idx].valid = 0;
                continue;
            }

            float px, py, pz;
            float prx, pry, prz;
            float pbx, pby, pbz;

            if (!reconstruct_point(x, y, w, h, r, &px, &py, &pz)) continue;
            if (!reconstruct_point(rx, y, w, h, range_img->data[y * w + rx], &prx, &pry, &prz)) continue;
            if (!reconstruct_point(x, ry, w, h, range_img->data[ry * w + x], &pbx, &pby, &pbz)) continue;

            // Tangent vectors
            float vx_x = prx - px;
            float vx_y = pry - py;
            float vx_z = prz - pz;

            float vy_x = pbx - px;
            float vy_y = pby - py;
            float vy_z = pbz - pz;

            // Cross product n = vx x vy
            float nx = vx_y * vy_z - vx_z * vy_y;
            float ny = vx_z * vy_x - vx_x * vy_z;
            float nz = vx_x * vy_y - vx_y * vy_x;

            float norm = sqrtf(nx*nx + ny*ny + nz*nz);
            if (norm == 0.0f || !isfinite(norm)) {
                normals[idx].valid = 0;
                continue;
            }

            normals[idx].nx = nx / norm;
            normals[idx].ny = ny / norm;
            normals[idx].nz = nz / norm;
            normals[idx].valid = 1;
        }
    }

    return normals;
}

int save_normal_map_as_pgm(Normal* normals, int width, int height, const char* filename)
{
    if (!normals || width <= 0 || height <= 0 || !filename) return 0;

    FILE* f = fopen(filename, "wb");
    if (!f) return 0;

    fprintf(f, "P5\n%d %d\n255\n", width, height);

    for (int i = 0; i < width * height; i++) {
        unsigned char pix = 0;
        if (normals[i].valid) {
            // Map Z component [-1,1] -> [0,255] (upwards -> brighter)
            float v = normals[i].nz;
            float val = (v * 0.5f) + 0.5f;
            if (val < 0.0f) val = 0.0f;
            if (val > 1.0f) val = 1.0f;
            pix = (unsigned char)(val * 255.0f);
        } else {
            pix = 0;
        }
        fwrite(&pix, 1, 1, f);
    }

    fclose(f);
    return 1;
}
