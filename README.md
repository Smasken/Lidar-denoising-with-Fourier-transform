# LiDAR Surface Normal Estimation and Ground Segmentation

Processes Velodyne HDL-64E point clouds from the KITTI dataset. For each scan
it produces a range image, a surface-normal map, a depth-discontinuity map, and
an optional ground-segmentation overlay.

## Build

Requires a C compiler and `libomp` (OpenMP runtime). On macOS install libomp
via Homebrew first:

```bash
brew install libomp
make
```

This produces the `denoise` binary.

## Usage

### Single-scan mode

Run on the default scan (hardcoded path), or pass an explicit `.bin` file:

```bash
./denoise
./denoise path/to/scan.bin
```

Outputs written to the current directory:
- `range_img.pgm` — preprocessed range image
- `normals.pgm` — surface normal map (brightness = upward-facing)
- `discontinuity.pgm` — depth discontinuity mask

Per-stage timing is printed to stdout.

### Batch mode

Pass a directory containing `.bin` files:

```bash
./denoise <input_dir> [images_dir] [videos_dir] [R] [N] [D] [G]
```

| Argument | Default | Description |
|---|---|---|
| `input_dir` | — | Directory of `.bin` LiDAR scans (required) |
| `images_dir` | `data/output_images` | Where per-frame PGM/PPM images are written |
| `videos_dir` | `data/output_videos` | Where compiled MP4 videos are written |
| `R` | `n` | `y` / `n` — create range video |
| `N` | `n` | `y` / `n` — create normals video |
| `D` | `n` | `y` / `n` — create discontinuity video |
| `G` | `n` | `y` / `n` — create ground-segmentation video |

The four `y`/`n` flags must appear together at the end of the argument list.
Omitting them disables all video creation (images are still written).
Video creation requires `ffmpeg` to be installed.

Per-frame images are always written regardless of the video flags:
- `range_NNNNN.pgm`
- `normals_NNNNN.pgm`
- `disc_NNNNN.pgm`
- `ground_NNNNN.ppm` (only when `G y` is set)

A timing summary (ms per stage, average Hz) is printed after the batch
completes.

### Thread count (`-t`)

Add `-t <N>` anywhere in the argument list to set the number of OpenMP threads:

```bash
./denoise path/to/scan.bin -t 4
./denoise data/velodyne_points/data -t 8 y y y y
```

If omitted, OpenMP uses its default (typically one thread per logical core).

## Examples

```bash
# Single scan, 4 threads
./denoise data/2011_09_26/velodyne_points/data/0000000000.bin -t 4

# Batch — write images only, 8 threads
./denoise data/2011_09_26/velodyne_points/data -t 8 n n n n

# Batch — write all images and all videos
./denoise data/2011_09_26/velodyne_points/data y y y y

# Batch — custom output dirs, ground segmentation video only
./denoise data/2011_09_26/velodyne_points/data my_imgs my_vids n n n y
```

## Ground segmentation parameters

Ground pixels must satisfy two conditions simultaneously:
- Normal z-component ≥ 0.9 (surface within ~26° of horizontal)
- Reconstructed point height ≤ 0.5 m above sensor plane

Ground pixels are coloured orange in the `ground_NNNNN.ppm` output.
