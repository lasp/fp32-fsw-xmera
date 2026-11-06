#include "regionsOfInterestPruneAlgorithm.h"

#include <math.h>
#include <algorithm>
#include <iterator>

RegionsOfInterestPruneAlgorithm::RegionsOfInterestPruneAlgorithm(const RegionsOfInterestPruneConfig& config)
    : cfg(config) {
    setConfig(config);
}

void RegionsOfInterestPruneAlgorithm::setConfig(const RegionsOfInterestPruneConfig& config) { this->cfg = config; }

/*! Update method for the regions-of-interest pruning algorithm.  Orchestrates
 *  the three pipeline steps and returns a fully populated RoiCandidates.
 @return RoiCandidates  Candidates sorted by estimated pixel count (descending), ties broken by
                        distance to the reference point (ascending) then window area (ascending).
 @param rowSums   Pointer to the per-row above-threshold pixel sums.
 @param numRows   Number of rows in the sum array.
 @param colSums   Pointer to the per-column above-threshold pixel sums.
 @param numCols   Number of columns in the sum array.
*/
RoiCandidates RegionsOfInterestPruneAlgorithm::update(const uint16_t* rowSums,
                                                      uint32_t numRows,
                                                      const uint16_t* colSums,
                                                      uint32_t numCols) const {
    // Reference point (x0 = col, y0 = row).
    const auto referenceRow = static_cast<float>(this->cfg.getY0());
    const auto referenceCol = static_cast<float>(this->cfg.getX0());

    // Step 1: locate contiguous non-zero spans, keeping only the spans closest to the reference point
    // to bound the cross-product size.
    const RowTopSpans rowSpans = findTopSpans<ROW_SPANS_MAX>(std::span<const uint16_t>(rowSums, numRows), referenceRow);
    const ColTopSpans colSpans = findTopSpans<COL_SPANS_MAX>(std::span<const uint16_t>(colSums, numCols), referenceCol);

    // Steps 2–3: form bounding boxes, sort by estimated pixel count, truncate.
    return packOutput(buildCandidates(rowSpans, colSpans), referenceRow, referenceCol);
}

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
 *  earlier span wins.
 @return TopSpans holding at most Capacity spans, sorted by distance ascending.
 @param s          Span over the 1-D sum array.
 @param reference  [px] Reference coordinate along this axis (reference row or reference col).
*/
template <uint32_t Capacity>
RegionsOfInterestPruneAlgorithm::TopSpans<Capacity> RegionsOfInterestPruneAlgorithm::findTopSpans(
    std::span<const uint16_t> s,
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
        while (pos < n && s[pos] != 0) {
            sum += s[pos];
            ++pos;
        }
        const uint32_t length = pos - spanStart;
        const float midpoint = static_cast<float>(spanStart) + (static_cast<float>(length) / 2.0F);
        const float distance = fabsf(midpoint - reference);
        const SpanEntry span{.start = spanStart, .length = length, .sum = sum, .distance = distance};

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
 *  obtainable from 1-D projections alone.
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
            candidates.data[candidates.count++] = {
                .row = r.start, .col = c.start, .height = r.length, .width = c.length, .count = std::min(r.sum, c.sum)};
        }
    }
    return candidates;
}

/*! Sorts candidates by estimated pixel count (descending); ties are broken first by squared distance
 *  from the candidate's box center to the reference point (ascending — closer wins), then by window
 *  area (ascending — smaller wins). Truncates to ROI_CANDIDATES_MAX and packs the result into a
 *  RoiCandidates ready for publication.
 @return RoiCandidates with numCandidates set and candidates[0] = rank-1.
 @param candidates    Unsorted candidate list (taken by value; sorted in-place).
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
                          if (a.count != b.count) {
                              return a.count > b.count;
                          }
                          const float distA = squaredDistanceToReference(a, referenceRow, referenceCol);
                          const float distB = squaredDistanceToReference(b, referenceRow, referenceCol);
                          if (distA != distB) {
                              return distA < distB;
                          }
                          return a.height * a.width < b.height * b.width;
                      });

    RoiCandidates outRoi{};
    outRoi.numCandidates = std::min(candidates.count, ROI_CANDIDATES_MAX);
    for (uint32_t i = 0; i < outRoi.numCandidates; ++i) {
        outRoi.candidates[i] = candidates.data[i];
    }
    return outRoi;
}
// NOLINTEND(bugprone-easily-swappable-parameters)
