#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>

#include "lidar.h"
#include "image.h"
#include "diffusion.h"
#include "fft.h"
#include "metrics.h"
#include "noise.h"
#include "normals.h"
#include "discontinuity.h"

// Ensure a directory exists (create if missing)
static int ensure_dir(const char* path)
{
    struct stat st = {0};
    if (stat(path, &st) == -1) {
        if (mkdir(path, 0755) != 0) return 0;
    }
    return 1;
}

// Process a single .bin input and save range, normals, and discontinuity images
static void process_and_save(const char* infile, int idx, const char* outdir)
{
    LidarData* scan = load_lidar_data(infile);
    if (!scan) {
        fprintf(stderr, "Failed to load %s\n", infile);
        return;
    }

    Image* range_img = lidar_to_range_image(scan, 1024, 64);
    if (!range_img) {
        free_lidar_data(scan);
        return;
    }

    fill_holes(range_img, 5);
    median_filter(range_img);

    char path[1024];
    snprintf(path, sizeof(path), "%s/range_%05d.pgm", outdir, idx);
    save_image_as_pgm(range_img, path);

    Normal* normals = compute_surface_normals(range_img);
    if (normals) {
        snprintf(path, sizeof(path), "%s/normals_%05d.pgm", outdir, idx);
        save_normal_map_as_pgm(normals, range_img->width, range_img->height, path);
        free(normals);
    }

    Image* disc = compute_depth_discontinuities(range_img, 1.0f);
    if (disc) {
        snprintf(path, sizeof(path), "%s/disc_%05d.pgm", outdir, idx);
        save_discontinuity_as_pgm(disc, path);
        free_image(disc);
    }

    free_image(range_img);
    free_lidar_data(scan);
}

// Run the original single-scan experiment pipeline (denoising comparisons)
static int run_single_experiment(const char* infile)
{
    LidarData* scan = load_lidar_data(infile);
    if (!scan) {
        printf("Failed to read LIDAR data\n");
        return 1;
    }

    printf("loaded %d points\n", scan->num_points);

    Image* range_img = lidar_to_range_image(scan, 1392, 512);
    if (!range_img) {
        free_lidar_data(scan);
        return 1;
    }

    printf("Created range image: %d x %d\n", range_img->width, range_img->height);

    fill_holes(range_img, 5);
    median_filter(range_img);
    save_image_as_pgm(range_img, "range_img.pgm");


    printf("\n--- Running experiment ---\n");

    // Also save normals and discontinuity for this scan
    Normal* normals = compute_surface_normals(range_img);
    if (normals) {
        save_normal_map_as_pgm(normals, range_img->width, range_img->height, "normals.pgm");
        free(normals);
    }
    Image* disc = compute_depth_discontinuities(range_img, 1.0f);
    if (disc) {
        save_discontinuity_as_pgm(disc, "discontinuity.pgm");
        free_image(disc);
    }

    // Cleanup
    free_image(range_img);
    free_lidar_data(scan);

    return 0;
}

int main(int argc, char** argv)
{
    // If user provides a directory, run batch mode. If user provides a .bin file, run single experiment.
    if (argc == 1) {
        // default single-file as before
        const char* default_file = "data/2011_09_26/velodyne_points/data/0000000000.bin";
        return run_single_experiment(default_file);
    }

    // If first argument is a directory -> batch mode
    struct stat st;
    if (stat(argv[1], &st) == 0 && S_ISDIR(st.st_mode)) {
        const char* input_dir = argv[1];
        const char* out_dir = (argc > 2) ? argv[2] : "output_frames";

        if (!ensure_dir(out_dir)) {
            fprintf(stderr, "Failed to create output directory %s\n", out_dir);
            return 1;
        }

        DIR* d = opendir(input_dir);
        if (!d) {
            fprintf(stderr, "Failed to open input directory: %s\n", input_dir);
            return 1;
        }

        struct dirent* entry;
        int idx = 0;
        // collect .bin names
        char** names = NULL;
        size_t names_cap = 0, names_len = 0;
        while ((entry = readdir(d)) != NULL) {
            size_t len = strlen(entry->d_name);
            if (len > 4 && strcmp(entry->d_name + len - 4, ".bin") == 0) {
                if (names_len + 1 > names_cap) {
                    names_cap = names_cap ? names_cap * 2 : 64;
                    names = realloc(names, names_cap * sizeof(char*));
                }
                names[names_len++] = strdup(entry->d_name);
            }
        }
        closedir(d);

        if (names_len == 0) {
            printf("No .bin files found in %s\n", input_dir);
            free(names);
            return 0;
        }

        // sort names
        for (size_t i = 0; i < names_len; i++) {
            for (size_t j = i + 1; j < names_len; j++) {
                if (strcmp(names[i], names[j]) > 0) {
                    char* t = names[i]; names[i] = names[j]; names[j] = t;
                }
            }
        }

        for (size_t i = 0; i < names_len; i++) {
            char fullpath[2048];
            snprintf(fullpath, sizeof(fullpath), "%s/%s", input_dir, names[i]);
            printf("Processing %s (%zu/%zu)\n", names[i], i+1, names_len);
            process_and_save(fullpath, idx, out_dir);
            idx++;
            free(names[i]);
        }
        free(names);

        printf("Batch processing complete. To assemble a video run:\n");
        printf("ffmpeg -y -framerate 10 -i %s/range_%%05d.pgm -c:v libx264 -pix_fmt yuv420p range_video.mp4\n", out_dir);

        return 0;
    }

    // Otherwise assume argv[1] is a file -> run single experiment on it
    return run_single_experiment(argv[1]);
}