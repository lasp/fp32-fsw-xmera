#include "regionsOfInterestPruneAlgorithm.h"

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
                        distance to image center (ascending) then window area (ascending).
 @param rowSums   Pointer to the per-row above-threshold pixel sums.
 @param numRows   Number of rows in the sum array.
 @param colSums   Pointer to the per-column above-threshold pixel sums.
 @param numCols   Number of columns in the sum array.
*/
RoiCandidates RegionsOfInterestPruneAlgorithm::update(const uint16_t* rowSums,
                                                      uint32_t numRows,
                                                      const uint16_t* colSums,
                                                      uint32_t numCols) const {
    // Step 1: locate contiguous non-zero spans, keeping only the highest-sum spans to bound the
    // cross-product size.
    const RowTopSpans rowSpans =
        findTopSpans<ROW_SPANS_MAX>(std::span<const uint16_t>(rowSums, numRows), this->cfg.getMaxRowSpans());
    const ColTopSpans colSpans =
        findTopSpans<COL_SPANS_MAX>(std::span<const uint16_t>(colSums, numCols), this->cfg.getMaxColSpans());

    // Steps 2–3: form bounding boxes, sort by estimated pixel count, truncate.
    return packOutput(buildCandidates(rowSpans, colSpans), numRows, numCols);
}

namespace {
/*! Squared Euclidean distance from a candidate box's center to a reference center point;
 *  only the ordering matters here, so the sqrt is skipped.
 @return Squared distance from the candidate's box center to (centerRow, centerCol).
 @param candidate Candidate box.
 @param centerRow Reference point row coordinate.
 @param centerCol Reference point column coordinate.
*/
// NOLINTBEGIN(bugprone-easily-swappable-parameters) -- centerRow/centerCol are a legitimately
// paired 2-D point (the reference center); this shape is intentional.
float squaredDistanceToCenter(const RoiCandidateEntry& candidate, float centerRow, float centerCol) {
    const float candidateCenterRow = static_cast<float>(candidate.row) + (static_cast<float>(candidate.height) / 2.0F);
    const float candidateCenterCol = static_cast<float>(candidate.col) + (static_cast<float>(candidate.width) / 2.0F);
    const float dRow = candidateCenterRow - centerRow;
    const float dCol = candidateCenterCol - centerCol;
    return (dRow * dRow) + (dCol * dCol);
}
// NOLINTEND(bugprone-easily-swappable-parameters)
}  // namespace

/*! Scans a 1-D sum array for contiguous non-zero spans and keeps the top-keep spans by accumulated
 *  sum in a single forward pass.  Each finished span is insertion-sorted into a fixed-capacity buffer
 *  (O(N * keep)); when the buffer is full, a span replaces the current smallest only if its sum is
 *  strictly greater, so on ties the earlier span wins.
 @return TopSpans holding at most keep spans, sorted by sum descending.
 @param s     Span over the 1-D sum array.
 @param keep  Maximum number of spans to retain (<= Capacity, enforced by the config).
*/
template <uint32_t Capacity>
RegionsOfInterestPruneAlgorithm::TopSpans<Capacity> RegionsOfInterestPruneAlgorithm::findTopSpans(
    std::span<const uint16_t> s,
    uint32_t keep) {
    TopSpans<Capacity> top;
    keep = std::min(keep, Capacity);
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
        const SpanEntry span{.start = spanStart, .length = pos - spanStart, .sum = sum};

        // Append if there is room, otherwise overwrite the smallest kept span if this one beats it.
        uint32_t slot = 0;
        if (top.count < keep) {
            slot = top.count++;
        } else if (sum > top.data[keep - 1].sum) {
            slot = keep - 1;
        } else {
            continue;
        }
        // Bubble the new span up past strictly smaller sums to keep data[0..count) sorted.
        while (slot > 0 && top.data[slot - 1].sum < sum) {
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
 @param rowSpans  Top row spans selected by findTopSpans (Step 1).
 @param colSpans  Top col spans selected by findTopSpans (Step 1).
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
 *  from the candidate's box center to the image center (ascending — closer wins), then by window area
 *  (ascending — smaller wins). Truncates to ROI_CANDIDATES_MAX and packs the result into a
 *  RoiCandidates ready for publication.
 @return RoiCandidates with numCandidates set and candidates[0] = rank-1.
 @param candidates  Unsorted candidate list (taken by value; sorted in-place).
 @param numRows     Number of rows in the image (defines the image center's row coordinate).
 @param numCols     Number of columns in the image (defines the image center's column coordinate).
*/
// NOLINTBEGIN(bugprone-easily-swappable-parameters) -- numRows/numCols are a legitimately paired
// image-size pair; this shape is intentional and matches update()'s own parameter order.
RoiCandidates RegionsOfInterestPruneAlgorithm::packOutput(CandidateArray candidates,
                                                          uint32_t numRows,
                                                          uint32_t numCols) {
    const float centerRow = static_cast<float>(numRows) / 2.0F;
    const float centerCol = static_cast<float>(numCols) / 2.0F;

    std::ranges::sort(candidates.data.begin(),
                      std::next(candidates.data.begin(), candidates.count),
                      [centerRow, centerCol](const RoiCandidateEntry& a, const RoiCandidateEntry& b) {
                          if (a.count != b.count) {
                              return a.count > b.count;
                          }
                          const float distA = squaredDistanceToCenter(a, centerRow, centerCol);
                          const float distB = squaredDistanceToCenter(b, centerRow, centerCol);
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
