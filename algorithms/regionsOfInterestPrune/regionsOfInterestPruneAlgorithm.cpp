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
 @return RoiCandidates  Candidates sorted by scaledIntensity (descending), ties broken by distance from the box
                        center to the reference point (ascending).
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
    const RowTopSpans rowSpans = findTopSpans<ROW_SPANS_MAX>(
        std::span<const uint16_t>(rowSums, numRows), std::span<const uint32_t>(rowIntensities, numRows), referenceRow);
    const ColTopSpans colSpans = findTopSpans<COL_SPANS_MAX>(
        std::span<const uint16_t>(colSums, numCols), std::span<const uint32_t>(colIntensities, numCols), referenceCol);

    // Step 2: form bounding boxes from the cross-product of the kept spans.
    const CandidateArray candidates = buildCandidates(rowSpans, colSpans);

    // Step 3: keep only boxes that overlap at least one lit tile, setting each box's count to the lit
    // pixels of the tiles it covers. A grid that does not cover this image cannot confirm any box, so it
    // yields no candidates.
    if (!isBinGridUsable(binGrid, numRows, numCols)) {
        return RoiCandidates{};
    }
    CandidateArray litCandidates = filterByLitTiles(candidates, binGrid);

    // Step 4: scale intensity by area^alpha, then sort by scaledIntensity (ties: closer to the reference point).
    scaleIntensityByArea(litCandidates, this->cfg.getAlphaAreaPruneScaling());
    return packOutput(litCandidates, referenceRow, referenceCol);
}
// NOLINTEND(bugprone-easily-swappable-parameters)

namespace {
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

/*! Scans a 1-D sum array for contiguous non-zero spans and keeps the Capacity spans whose midpoints
 *  (start + length/2) are closest to the reference coordinate, in a single forward pass.  Each
 *  finished span is insertion-sorted into a fixed-capacity buffer (O(N * Capacity)); when the buffer
 *  is full, a span replaces the current farthest only if it is strictly closer, so on ties the
 *  earlier span wins.  Spans are found from the counts; each span also sums the matching intensities.
 @return TopSpans holding at most Capacity spans, sorted by distance ascending.
 @param s          Span over the 1-D sum array.
 @param intensities  Span over the 1-D intensity array, same length as s.
 @param reference  [px] Reference coordinate along this axis (reference row or reference col).
*/
template <uint32_t Capacity>
RegionsOfInterestPruneAlgorithm::TopSpans<Capacity> RegionsOfInterestPruneAlgorithm::findTopSpans(
    std::span<const uint16_t> s,
    std::span<const uint32_t> intensities,
    float reference) {
    TopSpans<Capacity> top;
    const auto n = static_cast<uint32_t>(s.size());
    uint32_t pos = 0;
    while (pos < n) {
        if (s[pos] == 0) {
            ++pos;
            continue;
        }

        const uint32_t spanStart = pos;
        uint32_t sum = 0;
        uint64_t intensity = 0;  // 64-bit: a span of many rows can exceed uint32_t
        while (pos < n && s[pos] != 0) {
            sum += s[pos];
            intensity += intensities[pos];
            ++pos;
        }
        const uint32_t length = pos - spanStart;
        const float midpoint = static_cast<float>(spanStart) + (static_cast<float>(length) / 2.0F);
        const float distance = fabsf(midpoint - reference);
        const SpanEntry span{
            .start = spanStart, .length = length, .sum = sum, .intensity = intensity, .distance = distance};

        // Append if there is room, otherwise overwrite the farthest kept span if this one is closer.
        uint32_t slot = 0;
        if (top.count < Capacity) {
            slot = top.count++;
        } else if (distance < top.data[Capacity - 1].distance) {
            slot = Capacity - 1;
        } else {
            continue;
        }
        // Bubble the new span up past strictly farther spans to keep data[0..count) sorted.
        while (slot > 0 && top.data[slot - 1].distance > distance) {
            top.data[slot] = top.data[slot - 1];
            --slot;
        }
        top.data[slot] = span;
    }
    return top;
}

/*! Forms bounding-box candidates from the cross-product of the top row and col spans.
 *  The estimated pixel count for each box is min(R[k], C[l]) — the tightest upper bound
 *  obtainable from 1-D projections alone.  The estimated intensity is min(row-span intensity,
 *  col-span intensity) on the same reasoning; scaledIntensity is set later by scaleIntensityByArea.
 @return CandidateArray of RoiCandidateEntry, one per (row span × col span) pair.
 @param rowSpans  Closest row spans selected by findTopSpans (Step 1).
 @param colSpans  Closest col spans selected by findTopSpans (Step 1).
*/
RegionsOfInterestPruneAlgorithm::CandidateArray RegionsOfInterestPruneAlgorithm::buildCandidates(
    const RowTopSpans& rowSpans,
    const ColTopSpans& colSpans) {
    CandidateArray candidates;
    for (uint32_t a = 0; a < rowSpans.count; ++a) {
        const SpanEntry& r = rowSpans.data[a];
        for (uint32_t b = 0; b < colSpans.count; ++b) {
            const SpanEntry& c = colSpans.data[b];
            candidates.data[candidates.count++] = {.row = r.start,
                                                   .col = c.start,
                                                   .height = r.length,
                                                   .width = c.length,
                                                   .count = std::min(r.sum, c.sum),
                                                   .intensity = std::min(r.intensity, c.intensity)};
        }
    }
    return candidates;
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
 *  survivor's count is set to the sum of the lit counts of the tiles it covers (see sumLitTiles).
 @return CandidateArray holding only the candidates that touch a lit tile.
 @param candidates  Candidates from buildCandidates (Step 2).
 @param binGrid     Usable bin grid (see isBinGridUsable).
*/
RegionsOfInterestPruneAlgorithm::CandidateArray RegionsOfInterestPruneAlgorithm::filterByLitTiles(
    const CandidateArray& candidates,
    const RoiBinGrid& binGrid) {
    CandidateArray kept;
    for (uint32_t i = 0; i < candidates.count; ++i) {
        const uint32_t litPixels = sumLitTiles(candidates.data[i], binGrid);
        if (litPixels > 0U) {
            RoiCandidateEntry& entry = kept.data[kept.count++];
            entry = candidates.data[i];
            entry.count = litPixels;
        }
    }
    return kept;
}

/*! Sets each candidate's scaledIntensity = intensity / area^alpha, with area = width * height (always >= 1, since
 *  every span has at least one entry). alpha = 0 leaves scaledIntensity equal to the intensity.
 @param candidates             Candidates to update in place.
 @param alphaAreaPruneScaling  [-] Area exponent, in [MIN_ALPHA_AREA_PRUNE_SCALING, MAX_ALPHA_AREA_PRUNE_SCALING].
*/
void RegionsOfInterestPruneAlgorithm::scaleIntensityByArea(CandidateArray& candidates, float alphaAreaPruneScaling) {
    for (uint32_t i = 0; i < candidates.count; ++i) {
        RoiCandidateEntry& candidate = candidates.data[i];
        const auto area = static_cast<float>(candidate.width) * static_cast<float>(candidate.height);
        candidate.scaledIntensity = static_cast<float>(candidate.intensity) / powf(area, alphaAreaPruneScaling);
    }
}

/*! Sorts candidates by scaledIntensity (descending); ties are broken by squared distance from the
 *  candidate's box center to the reference point (ascending — closer wins). Packs the result into a
 *  RoiCandidates; the adapter publishes the first MAX_NUMBER_REGIONS.
 @return RoiCandidates with numCandidates set and candidates[0] = highest scaledIntensity.
 @param candidates    Unsorted candidate list with scaledIntensity set (taken by value; sorted in-place).
 @param referenceRow  [px] Reference point row coordinate (y0).
 @param referenceCol  [px] Reference point column coordinate (x0).
*/
// NOLINTBEGIN(bugprone-easily-swappable-parameters) -- referenceRow/referenceCol are a legitimately
// paired 2-D point; this shape is intentional.
RoiCandidates RegionsOfInterestPruneAlgorithm::packOutput(CandidateArray candidates,
                                                          float referenceRow,
                                                          float referenceCol) {
    std::ranges::sort(candidates.data.begin(),
                      std::next(candidates.data.begin(), candidates.count),
                      [referenceRow, referenceCol](const RoiCandidateEntry& a, const RoiCandidateEntry& b) {
                          if (a.scaledIntensity != b.scaledIntensity) {
                              return a.scaledIntensity > b.scaledIntensity;
                          }
                          return squaredDistanceToReference(a, referenceRow, referenceCol) <
                                 squaredDistanceToReference(b, referenceRow, referenceCol);
                      });

    RoiCandidates outRoi{};
    outRoi.numCandidates = std::min(candidates.count, ROI_CANDIDATES_MAX);
    for (uint32_t i = 0; i < outRoi.numCandidates; ++i) {
        outRoi.candidates[i] = candidates.data[i];
    }
    return outRoi;
}
// NOLINTEND(bugprone-easily-swappable-parameters)
