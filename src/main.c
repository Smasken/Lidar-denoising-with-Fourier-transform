#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>
#include <ctype.h>

#include "lidar.h"

/* Per-frame timing (milliseconds for each pipeline stage). */
typedef struct {
    double ms_projection;
    double ms_filtering;
    double ms_normals;
    double ms_segmentation;
    double ms_discontinuity;
    double ms_total;        /* wall time of full frame, excluding file I/O */
} FrameTiming;

static double elapsed_ms(struct timespec a, struct timespec b)
{
    return (b.tv_sec - a.tv_sec) * 1000.0
         + (b.tv_nsec - a.tv_nsec) / 1.0e6;
}
#include "image.h"
#include "diffusion.h"
#include "fft.h"
#include "metrics.h"
#include "noise.h"
#include "normals.h"
#include "discontinuity.h"
#include "segmentation.h"

// Ensure a directory exists (create if missing)
static int ensure_dir(const char* path)
{
    struct stat st = {0};
    if (stat(path, &st) == -1) {
        if (mkdir(path, 0755) != 0) return 0;
    }
    return 1;
}

// Process a single .bin input and save range, normals, discontinuity images
// If h_threshold >= 0, also save a ground segmentation overlay using that height threshold.
static FrameTiming process_and_save(const char* infile, int idx, const char* outdir, float h_threshold)
{
    FrameTiming ft = {0};
    struct timespec t0, t1, frame_start, frame_end;

    LidarData* scan = load_lidar_data(infile);
    if (!scan) {
        fprintf(stderr, "Failed to load %s\n", infile);
        return ft;
    }

    /* ---- projection ---- */
    clock_gettime(CLOCK_MONOTONIC, &frame_start);
    clock_gettime(CLOCK_MONOTONIC, &t0);
    Image* range_img = lidar_to_range_image(scan, 1024, 64);
    clock_gettime(CLOCK_MONOTONIC, &t1);
    ft.ms_projection = elapsed_ms(t0, t1);
    free_lidar_data(scan);

    if (!range_img) return ft;

    /* ---- hole-fill + median filter ---- */
    clock_gettime(CLOCK_MONOTONIC, &t0);
    fill_holes(range_img, 5);
    median_filter(range_img);
    clock_gettime(CLOCK_MONOTONIC, &t1);
    ft.ms_filtering = elapsed_ms(t0, t1);

    char path[1024];
    snprintf(path, sizeof(path), "%s/range_%05d.pgm", outdir, idx);
    save_image_as_pgm(range_img, path);

    /* ---- surface normals ---- */
    clock_gettime(CLOCK_MONOTONIC, &t0);
    Normal* normals = compute_surface_normals(range_img);
    clock_gettime(CLOCK_MONOTONIC, &t1);
    ft.ms_normals = elapsed_ms(t0, t1);

    if (normals) {
        snprintf(path, sizeof(path), "%s/normals_%05d.pgm", outdir, idx);
        save_normal_map_as_pgm(normals, range_img->width, range_img->height, path);

        /* ---- ground segmentation ---- */
        if (h_threshold >= 0.0f) {
            char segpath[1024];
            snprintf(segpath, sizeof(segpath), "%s/ground_%05d.ppm", outdir, idx);
            clock_gettime(CLOCK_MONOTONIC, &t0);
            // default nz threshold ~0.9 (≈25° from vertical), use provided h_threshold
            save_ground_overlay_as_ppm(normals, range_img, range_img->width, range_img->height, segpath, 0.9f, h_threshold);
            clock_gettime(CLOCK_MONOTONIC, &t1);
            ft.ms_segmentation = elapsed_ms(t0, t1);
        }
        free(normals);
    }

    /* ---- depth discontinuities ---- */
    clock_gettime(CLOCK_MONOTONIC, &t0);
    Image* disc = compute_depth_discontinuities(range_img, 1.0f);
    clock_gettime(CLOCK_MONOTONIC, &t1);
    ft.ms_discontinuity = elapsed_ms(t0, t1);

    if (disc) {
        snprintf(path, sizeof(path), "%s/disc_%05d.pgm", outdir, idx);
        save_discontinuity_as_pgm(disc, path);
        free_image(disc);
    }

    free_image(range_img);

    clock_gettime(CLOCK_MONOTONIC, &frame_end);
    ft.ms_total = elapsed_ms(frame_start, frame_end);
    return ft;
}

// Run the original single-scan experiment pipeline (denoising comparisons)
static int run_single_experiment(const char* infile)
{
    struct timespec t0, t1;

    LidarData* scan = load_lidar_data(infile);
    if (!scan) {
        printf("Failed to read LIDAR data\n");
        return 1;
    }

    printf("Loaded %d points\n", scan->num_points);

    clock_gettime(CLOCK_MONOTONIC, &t0);
    Image* range_img = lidar_to_range_image(scan, 1024, 64);
    clock_gettime(CLOCK_MONOTONIC, &t1);
    free_lidar_data(scan);
    if (!range_img) return 1;
    printf("Projection:    %6.2f ms\n", elapsed_ms(t0, t1));

    printf("Created range image: %d x %d\n", range_img->width, range_img->height);

    clock_gettime(CLOCK_MONOTONIC, &t0);
    fill_holes(range_img, 5);
    median_filter(range_img);
    clock_gettime(CLOCK_MONOTONIC, &t1);
    printf("Filtering:     %6.2f ms\n", elapsed_ms(t0, t1));
    save_image_as_pgm(range_img, "range_img.pgm");

    printf("\n--- Running experiment ---\n");

    clock_gettime(CLOCK_MONOTONIC, &t0);
    Normal* normals = compute_surface_normals(range_img);
    clock_gettime(CLOCK_MONOTONIC, &t1);
    printf("Normals:       %6.2f ms\n", elapsed_ms(t0, t1));
    if (normals) {
        save_normal_map_as_pgm(normals, range_img->width, range_img->height, "normals.pgm");
        free(normals);
    }

    clock_gettime(CLOCK_MONOTONIC, &t0);
    Image* disc = compute_depth_discontinuities(range_img, 1.0f);
    clock_gettime(CLOCK_MONOTONIC, &t1);
    printf("Discontinuity: %6.2f ms\n", elapsed_ms(t0, t1));
    if (disc) {
        save_discontinuity_as_pgm(disc, "discontinuity.pgm");
        free_image(disc);
    }

    free_image(range_img);
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
        const char* images_out_dir = "output_images";
        const char* videos_out_dir = "output_videos";

        // Parse optional flags: support either last-4 flags (r n d g) or last-3 (r n d)
        int create_range_video = 0, create_normals_video = 0, create_disc_video = 0, create_ground_video = 0;
        int flags_present = 0;

        if (argc >= 5) {
            int flags_start = argc - 4; // index of first possible flag (4 flags)
            if ((int)strlen(argv[flags_start]) == 1 && (int)strlen(argv[flags_start+1]) == 1 && (int)strlen(argv[flags_start+2]) == 1 && (int)strlen(argv[flags_start+3]) == 1) {
                char a = tolower((unsigned char)argv[flags_start][0]);
                char b = tolower((unsigned char)argv[flags_start+1][0]);
                char c = tolower((unsigned char)argv[flags_start+2][0]);
                char d = tolower((unsigned char)argv[flags_start+3][0]);
                if ((a == 'y' || a == 'n') && (b == 'y' || b == 'n') && (c == 'y' || c == 'n') && (d == 'y' || d == 'n')) {
                    create_range_video = (a == 'y');
                    create_normals_video = (b == 'y');
                    create_disc_video = (c == 'y');
                    create_ground_video = (d == 'y');
                    flags_present = 1;
                    if (flags_start > 2) images_out_dir = argv[2];
                    if (flags_start > 3) videos_out_dir = argv[3];
                }
            }
        }

        // Fallback: check for 3-flag form (no ground flag)
        if (!flags_present && argc >= 4) {
            int flags_start = argc - 3; // index of first possible flag (3 flags)
            if ((int)strlen(argv[flags_start]) == 1 && (int)strlen(argv[flags_start+1]) == 1 && (int)strlen(argv[flags_start+2]) == 1) {
                char a = tolower((unsigned char)argv[flags_start][0]);
                char b = tolower((unsigned char)argv[flags_start+1][0]);
                char c = tolower((unsigned char)argv[flags_start+2][0]);
                if ((a == 'y' || a == 'n') && (b == 'y' || b == 'n') && (c == 'y' || c == 'n')) {
                    create_range_video = (a == 'y');
                    create_normals_video = (b == 'y');
                    create_disc_video = (c == 'y');
                    create_ground_video = 0;
                    flags_present = 1;
                    if (flags_start > 2) images_out_dir = argv[2];
                    if (flags_start > 3) videos_out_dir = argv[3];
                }
            }
        }

        // If no flags present, allow optional argv[2]=images dir and argv[3]=videos dir
        if (!flags_present) {
            if (argc > 2) images_out_dir = argv[2];
            if (argc > 3) videos_out_dir = argv[3];
        }

        if (!ensure_dir(images_out_dir)) {
            fprintf(stderr, "Failed to create images output directory %s\n", images_out_dir);
            return 1;
        }
        if (!ensure_dir(videos_out_dir)) {
            fprintf(stderr, "Failed to create videos output directory %s\n", videos_out_dir);
            return 1;
        }

        DIR* d = opendir(input_dir);
        if (!d) {
            fprintf(stderr, "Failed to open input directory: %s\n", input_dir);
            return 1;
        }

        double total_ms = 0.0;
        double total_proj = 0.0, total_filt = 0.0, total_norm = 0.0;
        double total_seg  = 0.0, total_disc = 0.0;
        int frames_timed = 0;

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
            float h_threshold = create_ground_video ? 0.5f : -1.0f; // 0.5 m default when enabled
            FrameTiming ft = process_and_save(fullpath, idx, images_out_dir, h_threshold);
            if (ft.ms_total > 0.0) {
                total_ms   += ft.ms_total;
                total_proj += ft.ms_projection;
                total_filt += ft.ms_filtering;
                total_norm += ft.ms_normals;
                total_seg  += ft.ms_segmentation;
                total_disc += ft.ms_discontinuity;
                frames_timed++;
            }
            idx++;
            free(names[i]);
        }
        free(names);

        printf("Batch processing complete.\n");

        if (frames_timed > 0) {
            double avg_ms   = total_ms   / frames_timed;
            double avg_proj = total_proj / frames_timed;
            double avg_filt = total_filt / frames_timed;
            double avg_norm = total_norm / frames_timed;
            double avg_seg  = total_seg  / frames_timed;
            double avg_disc = total_disc / frames_timed;
            double compute_ms = avg_proj + avg_filt + avg_norm + avg_seg + avg_disc;
            printf("\n--- Timing summary (%d frames) ---\n", frames_timed);
            printf("  Projection:    %6.2f ms\n", avg_proj);
            printf("  Filtering:     %6.2f ms\n", avg_filt);
            printf("  Normals:       %6.2f ms\n", avg_norm);
            printf("  Segmentation:  %6.2f ms\n", avg_seg);
            printf("  Discontinuity: %6.2f ms\n", avg_disc);
            printf("  --------------------------------\n");
            printf("  Compute total: %6.2f ms  (%.1f Hz)\n", compute_ms, 1000.0 / compute_ms);
            printf("  Wall total:    %6.2f ms  (%.1f Hz, includes file I/O)\n", avg_ms, 1000.0 / avg_ms);
        }

        char cmd[2048];
        if (create_range_video) {
            snprintf(cmd, sizeof(cmd), "ffmpeg -y -framerate 10 -i %s/range_%%05d.pgm -c:v libx264 -pix_fmt yuv420p %s/range_video.mp4", images_out_dir, videos_out_dir);
            printf("Creating range video: %s\n", cmd);
            system(cmd);
        } else {
            printf("To create range video run:\nffmpeg -y -framerate 10 -i %s/range_%%05d.pgm -c:v libx264 -pix_fmt yuv420p %s/range_video.mp4\n", images_out_dir, videos_out_dir);
        }

        if (create_normals_video) {
            snprintf(cmd, sizeof(cmd), "ffmpeg -y -framerate 10 -i %s/normals_%%05d.pgm -c:v libx264 -pix_fmt yuv420p %s/normals_video.mp4", images_out_dir, videos_out_dir);
            printf("Creating normals video: %s\n", cmd);
            system(cmd);
        } else {
            printf("To create normals video run:\nffmpeg -y -framerate 10 -i %s/normals_%%05d.pgm -c:v libx264 -pix_fmt yuv420p %s/normals_video.mp4\n", images_out_dir, videos_out_dir);
        }

        if (create_disc_video) {
            snprintf(cmd, sizeof(cmd), "ffmpeg -y -framerate 10 -i %s/disc_%%05d.pgm -c:v libx264 -pix_fmt yuv420p %s/disc_video.mp4", images_out_dir, videos_out_dir);
            printf("Creating discontinuity video: %s\n", cmd);
            system(cmd);
        } else {
            printf("To create discontinuity video run:\nffmpeg -y -framerate 10 -i %s/disc_%%05d.pgm -c:v libx264 -pix_fmt yuv420p %s/disc_video.mp4\n", images_out_dir, videos_out_dir);
        }

        // Ground segmentation video
        if (create_ground_video) {
            snprintf(cmd, sizeof(cmd), "ffmpeg -y -framerate 10 -i %s/ground_%%05d.ppm -c:v libx264 -pix_fmt yuv420p %s/ground_video.mp4", images_out_dir, videos_out_dir);
            printf("Creating ground segmentation video: %s\n", cmd);
            system(cmd);
        } else {
            printf("To create ground segmentation video run:\nffmpeg -y -framerate 10 -i %s/ground_%%05d.ppm -c:v libx264 -pix_fmt yuv420p %s/ground_video.mp4\n", images_out_dir, videos_out_dir);
        }

        return 0;
    }

    // Otherwise assume argv[1] is a file -> run single experiment on it
    return run_single_experiment(argv[1]);
}