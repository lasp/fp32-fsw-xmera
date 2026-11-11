"""
Offline diagnostic visualizer for regionsOfInterestPrune.

Renders published RegionsIdentifiedMsgF32Payload candidates as an annotated PNG
over a background image, every region labeled with its rank and pixel count
("R<rank> (<numberOfPixels>)") plus a filled center dot: rank-1 in red, rank-2
in blue, all others in thin yellow.

Optionally also draws the bin grid's lit tiles (thin green outlines, the tiles the lit-tile
filter keeps boxes on) and the reference point (magenta cross), so a result can be checked
against the algorithm's inputs.
"""

import os

import cv2
import numpy as np

CENTER_DOT_RADIUS_PX = 3
LABEL_OFFSET_PX = 6
LABEL_MIN_Y = 12
LABEL_FONT_SCALE = 0.5
LABEL_TEXT_THICKNESS_PX = 1
ALL_REGIONS_THICKNESS_PX = 1
RANKED_THICKNESS_PX = 2

YELLOW = (0, 255, 255)  # OpenCV colors are BGR: B=0, G=255, R=255.
RED = (0, 0, 255)
BLUE = (255, 0, 0)
GREEN = (0, 200, 0)
MAGENTA = (255, 0, 255)
REFERENCE_MARKER_SIZE_PX = 15


def build_background(source_image):
    """Convert a grayscale (any bit depth) source image into an 8-bit BGR canvas."""
    img = source_image
    if img.dtype != np.uint8:
        img = cv2.normalize(img, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)


def draw_region(vis, region, color, thickness, label=None):
    """Draw one region's bounding box on `vis`; optionally add a filled center dot + label."""
    x = region.cornerX
    y = region.cornerY
    cv2.rectangle(vis, (x, y), (x + region.width, y + region.height), color, thickness)
    if label is not None:
        cv2.circle(vis, (x + region.width // 2, y + region.height // 2), CENTER_DOT_RADIUS_PX, color, cv2.FILLED)
        label_y = max(y - LABEL_OFFSET_PX, LABEL_MIN_Y)
        cv2.putText(
            vis, label, (x, label_y), cv2.FONT_HERSHEY_SIMPLEX,
            LABEL_FONT_SCALE, color, LABEL_TEXT_THICKNESS_PX, cv2.LINE_AA,
        )


def draw_lit_tiles(vis, lit_tiles, region_size):
    """Outline every bin-grid tile with a non-zero lit count (thin green)."""
    height, width = vis.shape[:2]
    for tile_row, tile_col in zip(*np.nonzero(lit_tiles)):
        x = int(tile_col) * region_size
        y = int(tile_row) * region_size
        cv2.rectangle(vis, (x, y), (min(x + region_size, width) - 1, min(y + region_size, height) - 1), GREEN, 1)


def draw_reference(vis, reference):
    """Mark the reference point (x0, y0) with a magenta cross."""
    cv2.drawMarker(vis, (int(reference[0]), int(reference[1])), MAGENTA, cv2.MARKER_CROSS,
                   REFERENCE_MARKER_SIZE_PX, 1)


def save_visualization(regions, source_image, time_tag, save_dir, lit_tiles=None, region_size=None,
                       reference=None):
    """Annotate `source_image` with `regions` and write "<time_tag>_pruning_output.png".

    Parameters
    ----------
    regions : sequence of objects with .cornerX, .cornerY, .width, .height, .numberOfPixels
              (as returned e.g. by _regions_from_log() in test_regionsOfInterestPrune.py),
              already sorted rank-1 first.
    source_image : 2-D numpy array (grayscale), used as the background canvas.
    time_tag : value embedded in the output filename.
    save_dir : directory the PNG is written into.
    lit_tiles : optional 2-D array of per-tile lit counts (tile rows x tile cols); lit tiles are outlined.
    region_size : tile size in pixels; required with lit_tiles.
    reference : optional (x0, y0) reference point to mark.

    Returns
    -------
    str or None : path to the written PNG, or None if there was nothing to draw (no regions and no
                  lit tiles / reference given).
    """
    if not regions and lit_tiles is None and reference is None:
        return None

    vis = build_background(source_image)
    if lit_tiles is not None:
        draw_lit_tiles(vis, lit_tiles, region_size)
    if reference is not None:
        draw_reference(vis, reference)

    for rank, reg in enumerate(regions, start=1):
        if rank == 1:
            color, thickness = RED, RANKED_THICKNESS_PX
        elif rank == 2:
            color, thickness = BLUE, RANKED_THICKNESS_PX
        else:
            color, thickness = YELLOW, ALL_REGIONS_THICKNESS_PX
        draw_region(vis, reg, color, thickness, label=f"R{rank} ({reg.numberOfPixels})")

    out_path = os.path.join(save_dir, f"{time_tag}_pruning_output.png")
    cv2.imwrite(out_path, vis)
    return out_path
