#include "regionsOfInterestPruneAlgorithm.h"

#include <math.h>
#include <algorithm>
#include <iterator>
#include <limits>

RegionsOfInterestPruneAlgorithm::RegionsOfInterestPruneAlgorithm(const RegionsOfInterestPruneConfig& config)
    : cfg(config) {
    setConfig(config);
}

void RegionsOfInterestPruneAlgorithm::setConfig(const RegionsOfInterestPruneConfig& config) { this->cfg = config; }

/*! Update method for the regions-of-interest pruning algorithm.  Orchestrates
 *  the pipeline steps and returns a fully populated RoiCandidates.
 @return RoiCandidates  Candidates sorted by distance from the box center to the reference point (ascending), ties
                        broken by scaledIntensity (descending).
 @param rowSums   Pointer to the per-row above-threshold pixel sums.
 @param numRows   Number of rows in the sum array.
 @param colSums   Pointer to the per-column above-threshold pixel sums.
 @param numCols   Number of columns in the sum array.
 @param rowIntensities  Pointer to the per-row intensity sums (length numRows).
 @param colIntensities  Pointer to the per-column intensity sums (length numCols).
 @param binGrid   Bin grid of lit tiles covering the same image; candidates touching no lit tile are dropped.
*/
// NOLINTBEGIN(bugprone-easily-swappable-parameters) -- rowIntensities/colIntensities pair with
// rowSums/colSums, in the same row-then-column order.
RoiCandidates RegionsOfInterestPruneAlgorithm::update(const uint16_t* rowSums,
                                                      uint32_t numRows,
                                                      const uint16_t* colSums,
                                                      uint32_t numCols,
                                                      const uint32_t* rowIntensities,
                                                      const uint32_t* colIntensities,
                                                      const RoiBinGrid& binGrid) const {
    // Reference point (x0 = col, y0 = row).
    const auto referenceRow = static_cast<float>(this->cfg.getY0());
    const auto referenceCol = static_cast<float>(this->cfg.getX0());

    // Step 1: locate contiguous non-zero spans, keeping only the spans closest to the reference point
    // to bound the cross-product size.
    const RowClosestSpans rowSpans = findClosestSpans<ROW_SPANS_MAX>(
        std::span<const uint16_t>(rowSums, numRows), std::span<const uint32_t>(rowIntensities, numRows), referenceRow);
    const ColClosestSpans colSpans = findClosestSpans<COL_SPANS_MAX>(
        std::span<const uint16_t>(colSums, numCols), std::span<const uint32_t>(colIntensities, numCols), referenceCol);

    // Step 2: form bounding boxes from the cross-product of the kept spans.
    const RoiCandidates allCandidates = buildCandidates(rowSpans, colSpans);

    // Step 2b: with 2 row x 2 col spans, drop the ghost boxes of the span pairing the projections reject.
    const RoiCandidates candidates = removeGhostPairs(allCandidates, rowSpans, colSpans);

    // Step 3: keep only boxes that overlap at least one lit tile, setting each box's litPixelCount to the lit
    // pixels of the tiles it covers. A grid that does not cover this image cannot confirm any box, so it
    // yields no candidates.
    if (!isBinGridUsable(binGrid, numRows, numCols)) {
        return RoiCandidates{};
    }
    RoiCandidates litCandidates = filterByLitTiles(candidates, binGrid);

    // Step 4: scale each box's intensity by area^alpha.
    scaleIntensityByArea(litCandidates, this->cfg.getAlphaAreaPruneScaling());

    // Step 5: sort by distance to the reference point, ties broken by scaledIntensity.
    rankCandidates(litCandidates, referenceRow, referenceCol);
    return litCandidates;
}
// NOLINTEND(bugprone-easily-swappable-parameters)

namespace {
/*! Relative mismatch |a - b| / max(a, b) in [0, 1]; 0 when both are zero.
 @return Relative mismatch of a and b.
 @param a  First non-negative value.
 @param b  Second non-negative value.
*/
float relativeMismatch(uint64_t a, uint64_t b) {
    const uint64_t larger = std::max(a, b);
    if (larger == 0U) {
        return 0.0F;
    }
    const uint64_t difference = (a > b) ? (a - b) : (b - a);
    return static_cast<float>(difference) / static_cast<float>(larger);
}

/*! Squared Euclidean distance from a candidate box's center to the reference point;
 *  only the ordering matters here, so the sqrt is skipped.
 @return Squared distance from the candidate's box center to (referenceRow, referenceCol).
 @param candidate    Candidate box.
 @param referenceRow Reference point row coordinate.
 @param referenceCol Reference point column coordinate.
*/
// NOLINTBEGIN(bugprone-easily-swappable-parameters) -- referenceRow/referenceCol are a legitimately
// paired 2-D point (the reference point); this shape is intentional.
float squaredDistanceToReference(const RoiCandidateEntry& candidate, float referenceRow, float referenceCol) {
    const float candidateCenterRow = static_cast<float>(candidate.row) + (static_cast<float>(candidate.height) / 2.0F);
    const float candidateCenterCol = static_cast<float>(candidate.col) + (static_cast<float>(candidate.width) / 2.0F);
    const float dRow = candidateCenterRow - referenceRow;
    const float dCol = candidateCenterCol - referenceCol;
    return (dRow * dRow) + (dCol * dCol);
}
// NOLINTEND(bugprone-easily-swappable-parameters)
}  // namespace

/*! Scans a 1-D count array for contiguous non-zero spans and keeps the Capacity spans whose midpoints
 *  (start + length/2) are closest to the reference coordinate, in a single forward pass.  Each
 *  finished span is insertion-sorted into a fixed-capacity buffer (O(N * Capacity)); when the buffer
 *  is full, a span replaces the current farthest only if it is strictly closer, so on ties the
 *  earlier span wins.  Spans are found from the counts; each span also sums the matching intensities.
 @return ClosestSpans holding at most Capacity spans, sorted by distance ascending.
 @param counts       Span over the 1-D above-threshold pixel count array.
 @param intensities  Span over the 1-D intensity array, same length as counts.
 @param reference  [px] Reference coordinate along this axis (reference row or reference col).
*/
template <uint32_t Capacity>
RegionsOfInterestPruneAlgorithm::ClosestSpans<Capacity> RegionsOfInterestPruneAlgorithm::findClosestSpans(
    std::span<const uint16_t> counts,
    std::span<const uint32_t> intensities,
    float reference) {
    ClosestSpans<Capacity> closest;
    const auto n = static_cast<uint32_t>(counts.size());
    uint32_t pos = 0;
    while (pos < n) {
        if (counts[pos] == 0) {
            ++pos;
            continue;
        }

        const uint32_t spanStart = pos;
        uint64_t pixels = 0;
        uint64_t intensity = 0;  // 64-bit: a span of many rows can exceed uint32_t
        while (pos < n && counts[pos] != 0) {
            pixels += counts[pos];
            intensity += intensities[pos];
            ++pos;
        }
        const uint32_t length = pos - spanStart;
        const float midpoint = static_cast<float>(spanStart) + (static_cast<float>(length) / 2.0F);
        const float distance = fabsf(midpoint - reference);
        const SpanEntry span{
            .start = spanStart, .length = length, .pixels = pixels, .intensity = intensity, .distance = distance};

        // Append if there is room, otherwise overwrite the farthest kept span if this one is closer.
        uint32_t slot = 0;
        if (closest.count < Capacity) {
            slot = closest.count++;
        } else if (distance < closest.data[Capacity - 1].distance) {
            slot = Capacity - 1;
        } else {
            continue;
        }
        // Bubble the new span up past strictly farther spans to keep data[0..count) sorted.
        while (slot > 0 && closest.data[slot - 1].distance > distance) {
            closest.data[slot] = closest.data[slot - 1];
            --slot;
        }
        closest.data[slot] = span;
    }
    return closest;
}

/*! Forms bounding-box candidates from the cross-product of the closest row and col spans.
 *  The estimated intensity for each box is min(row-span intensity, col-span intensity) — the tightest
 *  upper bound obtainable from 1-D projections alone. litPixelCount is set later by filterByLitTiles
 *  and scaledIntensity by scaleIntensityByArea.
 @return RoiCandidates with one entry per (row span × col span) pair.
 @param rowSpans  Closest row spans selected by findClosestSpans (Step 1).
 @param colSpans  Closest col spans selected by findClosestSpans (Step 1).
*/
RoiCandidates RegionsOfInterestPruneAlgorithm::buildCandidates(const RowClosestSpans& rowSpans,
                                                               const ColClosestSpans& colSpans) {
    RoiCandidates candidates;
    for (uint32_t a = 0; a < rowSpans.count; ++a) {
        const SpanEntry& r = rowSpans.data[a];
        for (uint32_t b = 0; b < colSpans.count; ++b) {
            const SpanEntry& c = colSpans.data[b];
            candidates.candidates[candidates.numCandidates++] = {.row = r.start,
                                                                 .col = c.start,
                                                                 .height = r.length,
                                                                 .width = c.length,
                                                                 .intensity = std::min(r.intensity, c.intensity)};
        }
    }
    return candidates;
}

/*! Removes ghost boxes when the kept spans form a 2 x 2 cross-product.
 *
 *  Two objects A and B whose rows and columns do not overlap give 2 row spans and 2 col spans, and the
 *  cross-product forms 4 boxes: the 2 real ones plus 2 "ghosts" that pair A's rows with B's columns and vice
 *  versa. The real boxes are one of the two ways to pair the spans one-to-one: the diagonal pairing
 *  {(row0, col0), (row1, col1)} or the anti-diagonal pairing {(row0, col1), (row1, col0)}.
 *
 *  A real box's row span and col span project the same object, so their above-threshold pixel counts agree, and
 *  so do their intensity sums; a ghost box pairs two different objects. Each pairing is scored by the summed
 *  relative mismatch of its two boxes (pixel counts + intensities), and the worse pairing's two boxes are dropped
 *  only when the better score is strictly below GHOST_PAIRING_MARGIN times the worse one. If the projections are
 *  ambiguous (e.g. two objects with the same pixel count and intensity) all 4 boxes are kept. With any other
 *  span layout (1 x 1, 1 x 2, 2 x 1) no ghost can form and the candidates are returned unchanged.
 @return Candidates with the ghost boxes removed, in their original order.
 @param candidates  Candidates from buildCandidates (Step 2), index = rowSpanIndex * colSpans.count + colSpanIndex.
 @param rowSpans    Closest row spans (Step 1).
 @param colSpans    Closest col spans (Step 1).
*/
RoiCandidates RegionsOfInterestPruneAlgorithm::removeGhostPairs(const RoiCandidates& candidates,
                                                                const RowClosestSpans& rowSpans,
                                                                const ColClosestSpans& colSpans) {
    if (rowSpans.count != 2U || colSpans.count != 2U) {
        return candidates;
    }
    const auto pairMismatch = [](const SpanEntry& row, const SpanEntry& col) {
        return relativeMismatch(row.pixels, col.pixels) + relativeMismatch(row.intensity, col.intensity);
    };
    const SpanEntry& row0 = rowSpans.data[0];
    const SpanEntry& row1 = rowSpans.data[1];
    const SpanEntry& col0 = colSpans.data[0];
    const SpanEntry& col1 = colSpans.data[1];
    const float diagonal = pairMismatch(row0, col0) + pairMismatch(row1, col1);      // boxes 0 and 3
    const float antiDiagonal = pairMismatch(row0, col1) + pairMismatch(row1, col0);  // boxes 1 and 2

    bool keepDiagonal = false;
    if (diagonal < GHOST_PAIRING_MARGIN * antiDiagonal) {
        keepDiagonal = true;
    } else if (!(antiDiagonal < GHOST_PAIRING_MARGIN * diagonal)) {
        return candidates;  // ambiguous: keep all four
    }

    RoiCandidates kept;
    for (uint32_t i = 0; i < candidates.numCandidates; ++i) {
        const bool isDiagonalBox = (i == 0U) || (i == 3U);
        if (isDiagonalBox == keepDiagonal) {
            kept.candidates[kept.numCandidates++] = candidates.candidates[i];
        }
    }
    return kept;
}

/*! Checks that the bin grid covers the image the candidates were built from: a non-zero tile size,
 *  ceil(numRows / regionSize) tile rows, ceil(numCols / regionSize) tile columns, and exactly one lit
 *  count per tile. Every candidate box then lies inside the grid, so the overlap lookup stays in bounds.
 @return true when the grid matches the image.
 @param binGrid  Bin grid to check.
 @param numRows  [px] Image height the row sums were taken over.
 @param numCols  [px] Image width the column sums were taken over.
*/
// NOLINTBEGIN(bugprone-easily-swappable-parameters) -- numRows/numCols are the paired image dimensions,
// in the same order as update().
bool RegionsOfInterestPruneAlgorithm::isBinGridUsable(const RoiBinGrid& binGrid, uint32_t numRows, uint32_t numCols) {
    if (binGrid.regionSize == 0U) {
        return false;
    }
    // 64-bit so the ceil-division cannot wrap for any regionSize.
    const uint64_t expectedRows = (static_cast<uint64_t>(numRows) + binGrid.regionSize - 1U) / binGrid.regionSize;
    const uint64_t expectedCols = (static_cast<uint64_t>(numCols) + binGrid.regionSize - 1U) / binGrid.regionSize;
    const uint64_t numTiles = static_cast<uint64_t>(binGrid.numRegionRows) * binGrid.numRegionCols;
    return binGrid.numRegionRows == expectedRows && binGrid.numRegionCols == expectedCols &&
           numTiles == binGrid.litCounts.size();
}
// NOLINTEND(bugprone-easily-swappable-parameters)

/*! Sums the lit counts of every tile a candidate box overlaps. Only the tiles the box covers are visited
 *  (rows row/regionSize .. (row+height-1)/regionSize, same for columns). Tiles at the box edge usually
 *  extend past it, so the sum can exceed the lit pixels inside the box itself.
 @return Sum of litCounts over the covered tiles, saturated at UINT32_MAX.
 @param candidate  Candidate box (non-empty, inside the grid; see isBinGridUsable).
 @param binGrid    Usable bin grid.
*/
uint32_t RegionsOfInterestPruneAlgorithm::sumLitTiles(const RoiCandidateEntry& candidate, const RoiBinGrid& binGrid) {
    const uint32_t firstTileRow = candidate.row / binGrid.regionSize;
    const uint32_t lastTileRow = (candidate.row + candidate.height - 1U) / binGrid.regionSize;
    const uint32_t firstTileCol = candidate.col / binGrid.regionSize;
    const uint32_t lastTileCol = (candidate.col + candidate.width - 1U) / binGrid.regionSize;
    uint64_t sum = 0;  // 64-bit so many large tile counts cannot wrap before saturating
    for (uint32_t tileRow = firstTileRow; tileRow <= lastTileRow; ++tileRow) {
        for (uint32_t tileCol = firstTileCol; tileCol <= lastTileCol; ++tileCol) {
            sum += binGrid.litCounts[(tileRow * binGrid.numRegionCols) + tileCol];
        }
    }
    return static_cast<uint32_t>(std::min<uint64_t>(sum, std::numeric_limits<uint32_t>::max()));
}

/*! Drops candidates whose box overlaps no lit tile, keeping the survivors in their original order. Each
 *  survivor's litPixelCount is set to the sum of the lit counts of the tiles it covers (see sumLitTiles).
 @return RoiCandidates holding only the candidates that touch a lit tile.
 @param candidates  Candidates from buildCandidates (Step 2).
 @param binGrid     Usable bin grid (see isBinGridUsable).
*/
RoiCandidates RegionsOfInterestPruneAlgorithm::filterByLitTiles(const RoiCandidates& candidates,
                                                                const RoiBinGrid& binGrid) {
    RoiCandidates kept;
    for (uint32_t i = 0; i < candidates.numCandidates; ++i) {
        const uint32_t litPixels = sumLitTiles(candidates.candidates[i], binGrid);
        if (litPixels > 0U) {
            RoiCandidateEntry& entry = kept.candidates[kept.numCandidates++];
            entry = candidates.candidates[i];
            entry.litPixelCount = litPixels;
        }
    }
    return kept;
}

/*! Sets each candidate's scaledIntensity = intensity / area^alpha, with area = width * height (always >= 1, since
 *  every span has at least one entry). alpha = 0 leaves scaledIntensity equal to the intensity.
 @param candidates             Candidates to update in place.
 @param alphaAreaPruneScaling  [-] Area exponent, in [MIN_ALPHA_AREA_PRUNE_SCALING, MAX_ALPHA_AREA_PRUNE_SCALING].
*/
void RegionsOfInterestPruneAlgorithm::scaleIntensityByArea(RoiCandidates& candidates, float alphaAreaPruneScaling) {
    for (uint32_t i = 0; i < candidates.numCandidates; ++i) {
        RoiCandidateEntry& candidate = candidates.candidates[i];
        const auto area = static_cast<float>(candidate.width) * static_cast<float>(candidate.height);
        candidate.scaledIntensity = static_cast<float>(candidate.intensity) / powf(area, alphaAreaPruneScaling);
    }
}

/*! Sorts candidates in place by squared distance from the candidate's box center to the reference point
 *  (ascending — closer wins); ties are broken by scaledIntensity (descending — brighter wins). Afterwards
 *  candidates[0] is the box closest to the reference point; the adapter publishes the first MAX_NUMBER_REGIONS.
 @param candidates    Candidate list with scaledIntensity set; sorted in place.
 @param referenceRow  [px] Reference point row coordinate (y0).
 @param referenceCol  [px] Reference point column coordinate (x0).
*/
// NOLINTBEGIN(bugprone-easily-swappable-parameters) -- referenceRow/referenceCol are a legitimately
// paired 2-D point; this shape is intentional.
void RegionsOfInterestPruneAlgorithm::rankCandidates(RoiCandidates& candidates,
                                                     float referenceRow,
                                                     float referenceCol) {
    std::ranges::sort(candidates.candidates.begin(),
                      std::next(candidates.candidates.begin(), candidates.numCandidates),
                      [referenceRow, referenceCol](const RoiCandidateEntry& a, const RoiCandidateEntry& b) {
                          const float distanceA = squaredDistanceToReference(a, referenceRow, referenceCol);
                          const float distanceB = squaredDistanceToReference(b, referenceRow, referenceCol);
                          if (distanceA != distanceB) {
                              return distanceA < distanceB;
                          }
                          return a.scaledIntensity > b.scaledIntensity;
                      });
}
// NOLINTEND(bugprone-easily-swappable-parameters)
