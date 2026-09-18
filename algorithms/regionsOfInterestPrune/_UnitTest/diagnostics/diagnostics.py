"""
Offline diagnostic visualizer for regionsOfInterestPrune.

Renders published RegionsIdentifiedMsgF32Payload candidates as an annotated PNG
over a background image: all published regions in thin cyan, rank-1 in red
(filled center dot + pixel-count label), rank-2 in blue.
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

CYAN = (0, 255, 255)
RED = (0, 0, 255)
BLUE = (255, 0, 0)


def build_background(source_image):
    """Convert a grayscale (any bit depth) source image into an 8-bit BGR canvas."""
    img = source_image
    if img.dtype != np.uint8:
        img = cv2.normalize(img, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)


def draw_region(vis, region, color, thickness, label=None):
    """Draw one region's bounding box on `vis`; optionally add a filled center dot + label."""
    x = region.centerX - region.width // 2
    y = region.centerY - region.height // 2
    cv2.rectangle(vis, (x, y), (x + region.width, y + region.height), color, thickness)
    if label is not None:
        cv2.circle(vis, (region.centerX, region.centerY), CENTER_DOT_RADIUS_PX, color, cv2.FILLED)
        label_y = max(y - LABEL_OFFSET_PX, LABEL_MIN_Y)
        cv2.putText(
            vis, label, (x, label_y), cv2.FONT_HERSHEY_SIMPLEX,
            LABEL_FONT_SCALE, color, LABEL_TEXT_THICKNESS_PX, cv2.LINE_AA,
        )


def save_visualization(regions, source_image, time_tag, save_dir):
    """Annotate `source_image` with `regions` and write "<time_tag>_pruning_output.png".

    Parameters
    ----------
    regions : sequence of objects with .centerX, .centerY, .width, .height, .numberOfPixels
              (as returned e.g. by _regions_from_log() in test_regionsOfInterestPrune.py),
              already sorted rank-1 first.
    source_image : 2-D numpy array (grayscale), used as the background canvas.
    time_tag : value embedded in the output filename.
    save_dir : directory the PNG is written into.

    Returns
    -------
    str or None : path to the written PNG, or None if there were no regions to draw.
    """
    if not regions:
        return None

    vis = build_background(source_image)

    for reg in regions:
        draw_region(vis, reg, CYAN, ALL_REGIONS_THICKNESS_PX)

    r1 = regions[0]
    draw_region(vis, r1, RED, RANKED_THICKNESS_PX, label=f"R1 ({r1.numberOfPixels})")
    if len(regions) >= 2:
        r2 = regions[1]
        draw_region(vis, r2, BLUE, RANKED_THICKNESS_PX, label=f"R2 ({r2.numberOfPixels})")

    out_path = os.path.join(save_dir, f"{time_tag}_pruning_output.png")
    cv2.imwrite(out_path, vis)
    return out_path
