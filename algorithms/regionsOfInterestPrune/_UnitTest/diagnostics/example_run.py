"""
example_run.py — drives the real, compiled RegionsOfInterestPruneAlgorithm on a real
image and renders the diagnostic visualization, without needing fpgaImagePipeline
(which doesn't exist in this repo).

There is no fpgaImagePipeline here to turn an image into row/col above-threshold
sums, so this script does that step itself in Python (mirroring the same
box-blur + percentile-threshold math test_regionsOfInterestPrune.py's
_auto_threshold() uses), then feeds the result into the real compiled algorithm.

Usage:
    python example_run.py [image_path] [--kernel 5] [--row-col-span 3] [--out-dir DIR]

Requires the regionsOfInterestPruneF32 module to be built and importable
(xmera.fp32.regionsOfInterestPruneF32), and its .i file to declare the
uint16Array / RoiCandidateEntryArray typemaps this script relies on.
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

from xmera.fp32 import regionsOfInterestPruneF32 as regionsOfInterestPrune


# ---------------------------------------------------------------------------
# Step 1-2: load the image, compute row/col above-threshold sums
# ---------------------------------------------------------------------------

def compute_row_col_sums(image_path, kernel_size=5, percentile=95.0):
    """Mirrors _auto_threshold()/FpgaImagePipeline's blur+threshold math, then
    collapses the binary threshold mask to row/col above-threshold pixel counts.

    Returns
    -------
    (background : 2-D uint16/uint8 ndarray, row_sums : uint16 ndarray[H],
     col_sums : uint16 ndarray[W])
    """
    img = cv2.imread(image_path, cv2.IMREAD_ANYDEPTH | cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(f"cv2 could not read: {image_path}")

    pixels = img.astype(np.float32) * (16.0 if img.dtype == np.uint8 else 1.0)
    blur = cv2.boxFilter(pixels, -1, (kernel_size, kernel_size), normalize=False)
    shift = {5: 1, 7: 2, 9: 3}.get(int(kernel_size), 1)
    blur_shifted = blur / float(1 << shift)

    threshold = max(1, int(np.percentile(blur_shifted.ravel(), percentile)))
    above = blur_shifted > threshold

    row_sums = above.sum(axis=1).astype(np.uint16)
    col_sums = above.sum(axis=0).astype(np.uint16)
    return img, row_sums, col_sums


# ---------------------------------------------------------------------------
# Step 3: run the real compiled algorithm
# ---------------------------------------------------------------------------

def _to_uint16_array(values, r):
    arr = r.new_uint16Array(len(values))
    for i, v in enumerate(values):
        r.uint16Array_setitem(arr, i, int(v))
    return arr


def run_pruning(row_sums, col_sums, row_col_span, r=regionsOfInterestPrune):
    """Runs RegionsOfInterestPruneAlgorithm.update() on real row/col sums.

    Returns the raw RoiCandidates result (candidates in corner-coordinate form).
    """
    cfg = r.RegionsOfInterestPruneConfig.create(row_col_span, row_col_span)
    algo = r.RegionsOfInterestPruneAlgorithm(cfg)

    row_arr = _to_uint16_array(row_sums, r)
    col_arr = _to_uint16_array(col_sums, r)
    try:
        return algo.update(row_arr, len(row_sums), col_arr, len(col_sums))
    finally:
        r.delete_uint16Array(row_arr)
        r.delete_uint16Array(col_arr)


# ---------------------------------------------------------------------------
# Step 4: convert corner-coordinate candidates -> center-coordinate regions
# ---------------------------------------------------------------------------

def to_center_coordinate_regions(candidates):
    """Same conversion RegionsOfInterestPrune::updateState() does: corner
    (row, col, width, height) -> center (centerX, centerY, width, height).

    Returns a list of SimpleNamespace, rank-1 (highest count) first, matching
    what _regions_from_log() hands to diagnostics.py in the unit test.
    """
    regions = []
    for k in range(candidates.numCandidates):
        c = candidates.candidates[k]
        regions.append(types.SimpleNamespace(
            centerX=c.col + c.width // 2,
            centerY=c.row + c.height // 2,
            width=c.width,
            height=c.height,
            numberOfPixels=c.count,
        ))
    return regions


# ---------------------------------------------------------------------------
# Driver
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("image_path", nargs="?", default=_DEFAULT_IMAGE)
    parser.add_argument("--kernel", type=int, default=5)
    parser.add_argument("--row-col-span", type=int, default=3)
    parser.add_argument("--out-dir", default=_HERE)
    args = parser.parse_args()

    background, row_sums, col_sums = compute_row_col_sums(args.image_path, args.kernel)
    candidates = run_pruning(row_sums, col_sums, args.row_col_span)
    regions = to_center_coordinate_regions(candidates)

    print(f"{len(regions)} candidate(s) identified.")
    for k, reg in enumerate(regions):
        print(f"  [{k}] center=({reg.centerX},{reg.centerY}) "
              f"size={reg.width}x{reg.height} count={reg.numberOfPixels}")

    time_tag = f"{args.row_col_span}_{os.path.splitext(os.path.basename(args.image_path))[0]}"
    out_path = diagnostics.save_visualization(regions, background, time_tag, args.out_dir)
    print(f"diagnostic visualization written to {out_path}")


if __name__ == "__main__":
    main()
