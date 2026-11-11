"""
example_run.py — runs the real, compiled RegionsOfInterestPrune module on an image and renders
the diagnostic visualization, without needing fpgaImagePipeline (which doesn't exist in this repo).

There is no fpgaImagePipeline here to turn an image into the FPGA products, so this script builds
them itself in Python, mirroring the FPGA math (box blur, shift, threshold):
    - row/col above-threshold pixel counts       -> FpgaRowColSumMsgF32Payload.rowSums / colSums
    - row/col sums of blurred values above threshold -> .rowIntensities / .colIntensities
    - per-tile above-threshold counts             -> FpgaBinGridMsgF32Payload (the bin grid)
It then writes those into the two input messages, runs RegionsOfInterestPrune (reset + updateState),
reads regionsIdentifiedOutMsg and draws the published regions, the lit tiles and the reference point.

Usage:
    python example_run.py IMAGE [IMAGE ...] (--threshold T | --percentile P) [--kernel 5]
                          [--region-size 128] [--alpha 0 0.5 1] [--x0 X --y0 Y] [--out-dir DIR]

The threshold must be chosen explicitly. It applies to the blurred image on the FPGA scale: an 8-bit
pixel is scaled by 16, summed over the kernel and shifted (kernel 5: x200 for a flat level, so
--threshold 2000 is roughly 8-bit level 10). --percentile P uses the P-th percentile of the blurred
image instead, which suits busy frames but breaks on mostly-black ones (a percentile of 0 gives a
threshold of 1, so every faint pixel is lit).

Images must fit the payload arrays: at most 4096 px wide and 3000 px tall. The reference point
defaults to the image center (the module default, 2048/1500, assumes a full 4096x3000 frame).
One PNG is written per image and alpha value.

Requires lasp-basilisk to be built and installed (xmera.fp32.regionsOfInterestPruneF32 and the F32
messages in xmera.architecture.messaging).
"""

import argparse
import inspect
import os
import types

import cv2
import numpy as np

_HERE = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
_DEFAULT_IMAGE = os.path.join(_HERE, "..", "pia_958_830.tiff")

# diagnostics.py lives right next to this script.
import diagnostics

from xmera.architecture import messaging
from xmera.fp32 import regionsOfInterestPruneF32 as regionsOfInterestPrune

MAX_ROWS = messaging.FPGA_ROW_COL_MAX_ROWS        # 3000
MAX_COLS = messaging.FPGA_ROW_COL_MAX_COLS        # 4096
MAX_TILES = messaging.FPGA_BIN_GRID_MAX_REGIONS   # 3008
BLUR_SHIFT = {5: 1, 7: 2, 9: 3}                   # kernel size -> right shift after the box sum


# ---------------------------------------------------------------------------
# Step 1: image -> FPGA products (Python mirror of the pipeline)
# ---------------------------------------------------------------------------

def load_image(image_path):
    img = cv2.imread(image_path, cv2.IMREAD_ANYDEPTH | cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(f"cv2 could not read: {image_path}")
    height, width = img.shape
    if height > MAX_ROWS or width > MAX_COLS:
        raise ValueError(f"{image_path}: {width}x{height} exceeds the {MAX_COLS}x{MAX_ROWS} payload limit")
    return img


def compute_fpga_products(img, kernel_size, region_size, percentile=None, threshold=None):
    """Blur, threshold, and reduce the image the way the FPGA does.

    8-bit images are scaled by 16 to approximate 12-bit sensor values, as before.
    The threshold is either given directly or taken as a percentile of the blurred image.

    Returns a SimpleNamespace with row_sums, col_sums (uint16), row_intensities, col_intensities
    (uint32), lit_tiles (2-D uint32, tile rows x tile cols), threshold and the above-threshold mask.
    """
    if kernel_size not in BLUR_SHIFT:
        raise ValueError("kernel must be 5, 7 or 9")
    pixels = img.astype(np.uint32) * (16 if img.dtype == np.uint8 else 1)
    box_sum = cv2.boxFilter(pixels.astype(np.float64), -1, (kernel_size, kernel_size), normalize=False)
    blurred = np.minimum(box_sum.astype(np.uint64) >> BLUR_SHIFT[kernel_size], 0xFFFF).astype(np.uint32)

    if threshold is None:
        threshold = max(1, int(np.percentile(blurred, percentile)))
    above = blurred > threshold
    lit_values = np.where(above, blurred, 0).astype(np.uint64)

    height, width = img.shape
    tile_rows = -(-height // region_size)
    tile_cols = -(-width // region_size)
    if tile_rows * tile_cols > MAX_TILES:
        raise ValueError(f"{tile_rows}x{tile_cols} tiles exceed FPGA_BIN_GRID_MAX_REGIONS={MAX_TILES}; "
                         "use a larger --region-size")
    lit_tiles = np.zeros((tile_rows, tile_cols), dtype=np.uint32)
    for tr in range(tile_rows):
        for tc in range(tile_cols):
            lit_tiles[tr, tc] = above[tr * region_size:(tr + 1) * region_size,
                                      tc * region_size:(tc + 1) * region_size].sum()

    return types.SimpleNamespace(
        row_sums=above.sum(axis=1).astype(np.uint16),
        col_sums=above.sum(axis=0).astype(np.uint16),
        row_intensities=np.minimum(lit_values.sum(axis=1), 0xFFFFFFFF).astype(np.uint32),
        col_intensities=np.minimum(lit_values.sum(axis=0), 0xFFFFFFFF).astype(np.uint32),
        lit_tiles=lit_tiles,
        threshold=threshold,
        above=above,
    )


# ---------------------------------------------------------------------------
# Step 2: FPGA products -> input messages
# ---------------------------------------------------------------------------

def build_row_col_msg(products):
    payload = messaging.FpgaRowColSumMsgF32Payload()
    payload.numRows = len(products.row_sums)
    payload.numCols = len(products.col_sums)
    payload.rowSums = [int(v) for v in products.row_sums]
    payload.colSums = [int(v) for v in products.col_sums]
    payload.rowIntensities = [int(v) for v in products.row_intensities]
    payload.colIntensities = [int(v) for v in products.col_intensities]
    return messaging.FpgaRowColSumMsgF32().write(payload)


def build_bin_grid_msg(products, region_size):
    tile_rows, tile_cols = products.lit_tiles.shape
    payload = messaging.FpgaBinGridMsgF32Payload()
    payload.regionSize = region_size
    payload.numRegionRows = tile_rows
    payload.numRegionCols = tile_cols
    payload.numRegions = tile_rows * tile_cols
    regions = []
    for tr in range(tile_rows):
        for tc in range(tile_cols):
            region = messaging.FpgaBinRegionF32()
            region.x = tc * region_size
            region.y = tr * region_size
            region.litPixelCount = int(products.lit_tiles[tr, tc])
            regions.append(region)
    payload.regions = regions
    return messaging.FpgaBinGridMsgF32().write(payload)


# ---------------------------------------------------------------------------
# Step 3: run the real compiled module
# ---------------------------------------------------------------------------

def run_module(row_col_msg, bin_grid_msg, x0, y0, alpha):
    """Runs RegionsOfInterestPrune exactly as a simulation would (reset, then one updateState)
    and returns the published regions, rank-1 first."""
    module = regionsOfInterestPrune.RegionsOfInterestPrune()
    module.x0 = x0
    module.y0 = y0
    module.alphaAreaPruneScaling = alpha
    module.rowColSumInMsg.subscribeTo(row_col_msg)
    module.binGridInMsg.subscribeTo(bin_grid_msg)
    module.reset(0)
    module.updateState(0)

    out = module.regionsIdentifiedOutMsg.read()
    return [
        types.SimpleNamespace(cornerX=out.cornerX[k], cornerY=out.cornerY[k], width=out.width[k],
                              height=out.height[k], numberOfPixels=out.numberOfPixels[k])
        for k in range(out.numberOfRegions)
    ]


# ---------------------------------------------------------------------------
# Driver
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("images", nargs="*", default=[_DEFAULT_IMAGE])
    parser.add_argument("--kernel", type=int, default=5, help="box blur size: 5, 7 or 9")
    threshold = parser.add_mutually_exclusive_group(required=True)
    threshold.add_argument("--threshold", type=int,
                           help="absolute threshold on the blurred image (FPGA scale, e.g. 2000)")
    threshold.add_argument("--percentile", type=float,
                           help="threshold = this percentile of the blurred image (e.g. 99)")
    parser.add_argument("--region-size", type=int, default=128, choices=(64, 128, 256), help="bin grid tile size")
    parser.add_argument("--alpha", type=float, nargs="+", default=[0.0],
                        help="alphaAreaPruneScaling value(s) in [-1, 1]; one PNG per value")
    parser.add_argument("--x0", type=int, help="reference column (default: image center)")
    parser.add_argument("--y0", type=int, help="reference row (default: image center)")
    parser.add_argument("--out-dir", default=_HERE)
    args = parser.parse_args()

    for alpha in args.alpha:
        if not regionsOfInterestPrune.RegionsOfInterestPruneConfig.isValidAlphaAreaPruneScaling(alpha):
            parser.error(f"--alpha {alpha} is outside [-1, 1]")  # invalid config would abort the process

    os.makedirs(args.out_dir, exist_ok=True)
    for image_path in args.images:
        img = load_image(image_path)
        height, width = img.shape
        x0 = width // 2 if args.x0 is None else args.x0
        y0 = height // 2 if args.y0 is None else args.y0
        products = compute_fpga_products(img, args.kernel, args.region_size,
                                         percentile=args.percentile, threshold=args.threshold)
        row_col_msg = build_row_col_msg(products)
        bin_grid_msg = build_bin_grid_msg(products, args.region_size)
        lit = int((products.lit_tiles > 0).sum())
        name = os.path.splitext(os.path.basename(image_path))[0]
        print(f"{name}: {width}x{height}, threshold={products.threshold}, "
              f"{lit}/{products.lit_tiles.size} tiles lit, reference=({x0}, {y0})")

        for alpha in args.alpha:
            regions = run_module(row_col_msg, bin_grid_msg, x0, y0, alpha)
            print(f"  alpha={alpha:g}: {len(regions)} region(s)")
            for rank, reg in enumerate(regions, start=1):
                print(f"    R{rank} corner=({reg.cornerX},{reg.cornerY}) size={reg.width}x{reg.height} "
                      f"litPixels={reg.numberOfPixels}")
            tag = f"{name}_alpha{alpha:g}"
            out_path = diagnostics.save_visualization(
                regions, img, tag, args.out_dir,
                lit_tiles=products.lit_tiles, region_size=args.region_size, reference=(x0, y0))
            print(f"    -> {out_path}")


if __name__ == "__main__":
    main()
