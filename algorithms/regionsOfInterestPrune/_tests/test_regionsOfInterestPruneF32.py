"""
Module tests for regionsOfInterestPruneF32: the adapter between the FPGA messages and the algorithm.

The algorithm itself is covered by the C++ tests in this directory. These tests check what the
adapter adds and what downstream modules rely on:

    test_message_connection        both inputs link and regionsIdentifiedOutMsg is written
    test_unlinked_input_raises     reset() rejects a missing rowColSumInMsg or binGridInMsg
    test_invalid_config_raises     reset() rejects x0, y0 or alphaAreaPruneScaling out of range
    test_single_object_published   corner, size, numberOfPixels and timeTag of one published region
    test_at_most_two_published     no more than the algorithm's 2 candidates are published
    test_closest_region_first      regions are published closest to (x0, y0) first
    test_reconfigure               reconfigure() applies a new reference point without a reset
    test_bin_grid_mismatch         a bin grid that does not cover the image publishes no regions
    test_pixel_count_clamped       a saturated litPixelCount is clamped to the int range

The input messages are written directly from Python (no fpgaImagePipeline), so the tests control
exactly what the module sees: per-row/col counts and intensities and the per-tile lit counts.
"""

import numpy as np
import pytest

from xmera.architecture import messaging
from xmera.fp32 import regionsOfInterestPruneF32 as regionsOfInterestPrune
from xmera.utilities import SimulationBaseClass, macros

MAX_ROWS = 3000  # FPGA_ROW_COL_MAX_ROWS
MAX_COLS = 4096  # FPGA_ROW_COL_MAX_COLS
MAX_TILES = 3008  # FPGA_BIN_GRID_MAX_REGIONS
MAX_NUMBER_REGIONS = messaging.MAX_NUMBER_REGIONS
CANDIDATES_OUT_MAX = 2  # ROI_CANDIDATES_OUT_MAX in regionsOfInterestPruneAlgorithm.h
INT_MAX = 2**31 - 1

# Small test image: 200 x 300 px, 50 px tiles, reference point at its center.
NUM_ROWS = 200
NUM_COLS = 300
REGION_SIZE = 50
X0 = NUM_COLS // 2
Y0 = NUM_ROWS // 2

STEP_SEC = 1.0


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def make_frame(objects, num_rows=NUM_ROWS, num_cols=NUM_COLS, region_size=REGION_SIZE):
    """Build the two FPGA input payloads for an image holding rectangular objects.

    objects: list of (row, col, height, width, value); every pixel of an object is above threshold with
    blurred value `value`. Returns (rowColSumPayload, binGridPayload).
    """
    image = np.zeros((num_rows, num_cols), dtype=np.uint32)
    for row, col, height, width, value in objects:
        image[row:row + height, col:col + width] = value
    lit = image > 0

    row_col = messaging.FpgaRowColSumMsgF32Payload()
    row_col.numRows = num_rows
    row_col.numCols = num_cols
    row_col.rowSums = _padded(lit.sum(axis=1), MAX_ROWS)
    row_col.colSums = _padded(lit.sum(axis=0), MAX_COLS)
    row_col.rowIntensities = _padded(image.sum(axis=1), MAX_ROWS)
    row_col.colIntensities = _padded(image.sum(axis=0), MAX_COLS)

    tile_rows = -(-num_rows // region_size)
    tile_cols = -(-num_cols // region_size)
    tiles = []
    for tile_row in range(tile_rows):
        for tile_col in range(tile_cols):
            tile = messaging.FpgaBinRegionF32()
            tile.x = tile_col * region_size
            tile.y = tile_row * region_size
            tile.litPixelCount = int(lit[tile.y:tile.y + region_size, tile.x:tile.x + region_size].sum())
            tiles.append(tile)
    bin_grid = messaging.FpgaBinGridMsgF32Payload()
    bin_grid.regionSize = region_size
    bin_grid.numRegionRows = tile_rows
    bin_grid.numRegionCols = tile_cols
    bin_grid.numRegions = len(tiles)
    bin_grid.regions = tiles + [messaging.FpgaBinRegionF32() for _ in range(MAX_TILES - len(tiles))]
    return row_col, bin_grid


def _padded(values, length):
    out = [0] * length
    out[:len(values)] = [int(v) for v in values]
    return out


def make_pruner(x0=X0, y0=Y0, alpha=1.0):
    pruner = regionsOfInterestPrune.RegionsOfInterestPrune()
    pruner.ModelTag = "regionsOfInterestPrune"
    pruner.x0 = x0
    pruner.y0 = y0
    pruner.alphaAreaPruneScaling = alpha
    return pruner


def make_sim(pruner, frame, link_row_col=True, link_bin_grid=True):
    """Sim with the pruner fed by standalone messages carrying `frame`; returns (sim, recorder, messages).

    The messages are returned so they stay alive as long as the sim does.
    """
    row_col, bin_grid = frame
    row_col_msg = messaging.FpgaRowColSumMsgF32().write(row_col)
    bin_grid_msg = messaging.FpgaBinGridMsgF32().write(bin_grid)
    if link_row_col:
        pruner.rowColSumInMsg.subscribeTo(row_col_msg)
    if link_bin_grid:
        pruner.binGridInMsg.subscribeTo(bin_grid_msg)

    sim = SimulationBaseClass.SimBaseClass()
    process = sim.CreateNewProcess("testProcess")
    process.addTask(sim.CreateNewTask("testTask", macros.sec2nano(STEP_SEC)))
    sim.AddModelToTask("testTask", pruner)
    recorder = pruner.regionsIdentifiedOutMsg.recorder()
    sim.AddModelToTask("testTask", recorder)
    return sim, recorder, (row_col_msg, bin_grid_msg)


def run(sim, steps=1):
    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(STEP_SEC * (steps - 1)))
    sim.ExecuteSimulation()


def published(recorder, step=-1):
    """Regions published at `step` as dicts, rank 1 first."""
    return [
        {
            "cornerX": recorder.cornerX[step][k],
            "cornerY": recorder.cornerY[step][k],
            "width": recorder.width[step][k],
            "height": recorder.height[step][k],
            "numberOfPixels": recorder.numberOfPixels[step][k],
            "timeTag": recorder.timeTag[step][k],
        }
        for k in range(recorder.numberOfRegions[step])
    ]


# ---------------------------------------------------------------------------
# Wiring and configuration
# ---------------------------------------------------------------------------


def test_message_connection():
    """Both inputs link, and regionsIdentifiedOutMsg is written on the first step."""
    pruner = make_pruner()
    sim, recorder, _msgs = make_sim(pruner, make_frame([(95, 145, 10, 10, 200)]))

    assert pruner.rowColSumInMsg.isLinked()
    assert pruner.binGridInMsg.isLinked()
    run(sim)
    assert len(recorder.numberOfRegions) == 1
    assert recorder.numberOfRegions[0] == 1


@pytest.mark.parametrize("missing", ["rowColSumInMsg", "binGridInMsg"])
def test_unlinked_input_raises(missing):
    """reset() names the input that is not connected."""
    pruner = make_pruner()
    sim, _recorder, _msgs = make_sim(pruner,
                                     make_frame([]),
                                     link_row_col=(missing != "rowColSumInMsg"),
                                     link_bin_grid=(missing != "binGridInMsg"))
    with pytest.raises(RuntimeError, match=missing):
        sim.InitializeSimulation()


@pytest.mark.parametrize("field, value, message", [
    ("x0", MAX_COLS + 1, "x0"),
    ("y0", MAX_ROWS + 1, "y0"),
    ("alphaAreaPruneScaling", 5.0, "alphaAreaPruneScaling"),
    ("alphaAreaPruneScaling", 2.5, "alphaAreaPruneScaling"),
    ("alphaAreaPruneScaling", float("inf"), "alphaAreaPruneScaling"),
    ("alphaAreaPruneScaling", float("nan"), "alphaAreaPruneScaling"),
])
def test_invalid_config_raises(field, value, message):
    """reset() rejects a reference point outside the full image, or an alpha above 2 or NaN (any negative alpha
    is valid: it bypasses the cost function)."""
    pruner = make_pruner()
    setattr(pruner, field, value)
    sim, _recorder, _msgs = make_sim(pruner, make_frame([]))
    with pytest.raises(RuntimeError, match=message):
        sim.InitializeSimulation()


# ---------------------------------------------------------------------------
# Published regions
# ---------------------------------------------------------------------------


def test_single_object_published():
    """One object: the published region carries its top-left corner (cornerX = column,
    cornerY = row), its size, the lit pixels of the tiles it covers, and the step time.

    The 10 x 12 object at row 95, col 145 crosses the tile borders at row 100 and col 150, so it
    covers 4 tiles holding all of its 120 lit pixels.
    """
    pruner = make_pruner()
    sim, recorder, _msgs = make_sim(pruner, make_frame([(95, 145, 10, 12, 200)]))
    run(sim, steps=2)

    regions = published(recorder)
    assert len(regions) == 1
    region = regions[0]
    assert (region["cornerX"], region["cornerY"]) == (145, 95)
    assert (region["width"], region["height"]) == (12, 10)
    assert region["numberOfPixels"] == 120
    assert region["timeTag"] == pytest.approx(STEP_SEC)


def test_at_most_two_published():
    """Four objects in separate rows and cols give 4 boxes per pairing; the module publishes at most
    the algorithm's 2 candidates and leaves the remaining slots zero."""
    objects = [(20, 20, 10, 10, 200), (20, 250, 10, 10, 200), (160, 20, 10, 10, 200), (160, 250, 10, 10, 200)]
    pruner = make_pruner()
    sim, recorder, _msgs = make_sim(pruner, make_frame(objects))
    run(sim)

    assert recorder.numberOfRegions[0] == CANDIDATES_OUT_MAX
    for k in range(CANDIDATES_OUT_MAX, MAX_NUMBER_REGIONS):
        assert recorder.width[0][k] == 0 and recorder.height[0][k] == 0 and recorder.numberOfPixels[0][k] == 0


def test_closest_region_first():
    """Two objects on a diagonal: the one closer to (x0, y0) is published first, even though the
    other is brighter and larger."""
    near = (90, 160, 10, 10, 100)  # center (95, 165), 18 px from (100, 150)
    far = (20, 20, 30, 30, 250)  # center (35, 35), brighter and 9x the area
    pruner = make_pruner()
    sim, recorder, _msgs = make_sim(pruner, make_frame([near, far]))
    run(sim)

    regions = published(recorder)
    assert len(regions) == 2
    assert (regions[0]["cornerX"], regions[0]["cornerY"]) == (160, 90)
    assert (regions[1]["cornerX"], regions[1]["cornerY"]) == (20, 20)


def test_reconfigure():
    """reconfigure() pushes a new reference point into the running algorithm: moving (x0, y0) onto
    the far object makes it rank 1 on the next step."""
    near = (90, 160, 10, 10, 100)
    far = (20, 20, 30, 30, 250)
    pruner = make_pruner()
    sim, recorder, _msgs = make_sim(pruner, make_frame([near, far]))
    run(sim)
    assert (recorder.cornerX[0][0], recorder.cornerY[0][0]) == (160, 90)

    pruner.x0 = 35
    pruner.y0 = 35
    pruner.reconfigure()
    sim.ConfigureStopTime(macros.sec2nano(STEP_SEC))
    sim.ExecuteSimulation()
    assert (recorder.cornerX[-1][0], recorder.cornerY[-1][0]) == (20, 20)


def test_bin_grid_mismatch():
    """A bin grid whose tile layout does not cover the image cannot confirm any box, so nothing is
    published."""
    row_col, bin_grid = make_frame([(95, 145, 10, 10, 200)])
    bin_grid.numRegionRows += 1
    pruner = make_pruner()
    sim, recorder, _msgs = make_sim(pruner, (row_col, bin_grid))
    run(sim)

    assert recorder.numberOfRegions[0] == 0


def test_pixel_count_clamped():
    """litPixelCount saturates at UINT32_MAX; the published int numberOfPixels is clamped to INT_MAX
    instead of wrapping negative."""
    row_col, bin_grid = make_frame([(95, 145, 10, 12, 200)])
    tiles = list(bin_grid.regions)
    tile_index = (95 // REGION_SIZE) * (-(-NUM_COLS // REGION_SIZE)) + (145 // REGION_SIZE)
    tiles[tile_index].litPixelCount = 2**32 - 1
    bin_grid.regions = tiles
    pruner = make_pruner()
    sim, recorder, _msgs = make_sim(pruner, (row_col, bin_grid))
    run(sim)

    assert recorder.numberOfPixels[0][0] == INT_MAX
