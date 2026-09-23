#include "regionsOfInterestPruneAlgorithm.h"

#include <algorithm>
#include <functional>
#include <iterator>
#include <numeric>

RegionsOfInterestPruneAlgorithm::RegionsOfInterestPruneAlgorithm(const RegionsOfInterestPruneConfig& config)
    : cfg(config) {
    setConfig(config);
}

void RegionsOfInterestPruneAlgorithm::setConfig(const RegionsOfInterestPruneConfig& config) { this->cfg = config; }

/*! Update method for the regions-of-interest pruning algorithm.  Orchestrates
 *  the four pipeline steps and returns a fully populated RoiCandidates.
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
    // Step 1: locate contiguous non-zero spans and accumulate per-span pixel sums.
    const auto [rowSpans, rowAccum] = findSpans(rowSums, numRows);
    const auto [colSpans, colAccum] = findSpans(colSums, numCols);

    // Step 2: keep only the highest-sum spans to bound the cross-product size.
    const auto topRows = topIndices(rowAccum, this->cfg.getMaxRowSpans());
    const auto topCols = topIndices(colAccum, this->cfg.getMaxColSpans());

    // Steps 3–4: form bounding boxes, sort by estimated pixel count, truncate.
    return packOutput(buildCandidates(rowSpans, rowAccum, topRows, colSpans, colAccum, topCols), numRows, numCols);
}

namespace {
/*! Squared Euclidean distance from a candidate box's center to a reference center point.  Squared
 *  (rather than true) distance preserves the same ordering while avoiding a sqrt per comparison.
 @return Squared distance from the candidate's box center to (centerRow, centerCol).
 @param e         Candidate box.
 @param centerRow Reference point row coordinate.
 @param centerCol Reference point column coordinate.
*/
float squaredDistanceToCenter(const RoiCandidateEntry& e, float centerRow, float centerCol) {
    const float rowCenter = static_cast<float>(e.row) + (static_cast<float>(e.height) / 2.0F);
    const float colCenter = static_cast<float>(e.col) + (static_cast<float>(e.width) / 2.0F);
    const float dRow = rowCenter - centerRow;
    const float dCol = colCenter - centerCol;
    return (dRow * dRow) + (dCol * dCol);
}
}  // namespace

/*! Scans a 1-D sum array and returns all contiguous non-zero spans together
 *  with the accumulated sum for each span.  A single forward pass produces both.
 @return Pair of (spans, accum) where spans[i] = (start, length) and accum[i] = sum over that span.
 @param s  Pointer to the 1-D sum array.
 @param n  Length of the array.
*/
// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic) -- s is a raw C array mirroring the
// message payload's void* pointer field; indexing it directly is the natural, minimal representation.
std::pair<RegionsOfInterestPruneAlgorithm::SpanArray, RegionsOfInterestPruneAlgorithm::AccumArray>
RegionsOfInterestPruneAlgorithm::findSpans(const uint16_t* s, uint32_t n) {
    SpanArray spans;
    AccumArray accum;
    for (uint32_t i = 0; i < n;) {
        if (s[i] != 0) {
            uint32_t j = i;
            uint32_t sum = 0;
            while (j < n && s[j] != 0) {
                sum += s[j++];
            }
            if (spans.count < MAX_SPANS) {
                spans.data[spans.count] = {i, j - i};
                accum.data[accum.count] = sum;
                ++spans.count;
                ++accum.count;
            }
            i = j;
        } else {
            ++i;
        }
    }
    return {spans, accum};
}
// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)

/*! Returns the indices of the top-keep entries of vals ordered by descending value.
 *  Uses std::ranges::partial_sort so only the retained portion is fully sorted (O(N log keep)).
 @return AccumArray of at most keep indices into vals, sorted by vals[i] descending.
 @param vals  Accumulator values to rank.
 @param keep  Maximum number of top entries to retain.
*/
RegionsOfInterestPruneAlgorithm::AccumArray RegionsOfInterestPruneAlgorithm::topIndices(const AccumArray& vals,
                                                                                        uint32_t keep) {
    AccumArray idx;
    // vals.count <= MAX_SPANS is guaranteed because AccumArray values are produced by
    // findSpans(), which caps the number of stored entries at MAX_SPANS.
    const uint32_t n = vals.count;
    // std::ranges::iota isn't implemented in this project's libc++ yet, despite being valid
    // C++23; plain std::iota is the portable choice.
    // NOLINTNEXTLINE(modernize-use-ranges)
    std::iota(idx.data.begin(), std::next(idx.data.begin(), n), 0);  // only initialize n valid slots
    keep = std::min(keep, n);                                        // use logical count
    std::ranges::partial_sort(idx.data.begin(),
                              std::next(idx.data.begin(), keep),
                              std::next(idx.data.begin(), n),
                              std::greater{},
                              [&vals](uint32_t i) { return vals.data[i]; });  // sort only valid slots
    idx.count = keep;                                                         // record logical size

    return idx;
}

/*! Forms bounding-box candidates from the cross-product of the filtered row and col spans.
 *  The estimated pixel count for each box is min(R[k], C[l]) — the tightest upper bound
 *  obtainable from 1-D projections alone.
 @return Vector of RoiCandidateEntry, one per (rowIdx × colIdx) pair.
 @param rowSpans  All detected row spans.
 @param R         Per-row-span accumulator sums.
 @param rowIdx    Indices into rowSpans / R selected by the pre-filter (Step 2).
 @param colSpans  All detected col spans.
 @param C         Per-col-span accumulator sums.
 @param colIdx    Indices into colSpans / C selected by the pre-filter (Step 2).
*/
// NOLINTBEGIN(bugprone-easily-swappable-parameters) -- R/rowIdx and C/colIdx are legitimately paired,
// documented adjacent inputs (an accumulator plus its pre-filtered index list); this shape is intentional.
RegionsOfInterestPruneAlgorithm::CandidateArray RegionsOfInterestPruneAlgorithm::buildCandidates(
    const SpanArray& rowSpans,
    const AccumArray& R,
    const AccumArray& rowIdx,
    const SpanArray& colSpans,
    const AccumArray& C,
    const AccumArray& colIdx) {
    CandidateArray candidates;
    for (uint32_t a = 0; a < rowIdx.count; ++a) {
        const uint32_t ki = rowIdx.data[a];
        for (uint32_t b = 0; b < colIdx.count; ++b) {
            const uint32_t li = colIdx.data[b];
            const auto [r, h] = rowSpans.data[ki];
            const auto [c, w] = colSpans.data[li];
            candidates.data[candidates.count++] = {
                .row = r, .col = c, .height = h, .width = w, .count = std::min(R.data[ki], C.data[li])};
        }
    }
    return candidates;
}
// NOLINTEND(bugprone-easily-swappable-parameters)

/*! Sorts candidates by estimated pixel count (descending); ties are broken first by squared distance
 *  from the candidate's box center to the image center (ascending — closer wins), then by window area
 *  (ascending — smaller wins). Truncates to ROI_CANDIDATES_MAX and packs the result into a
 *  RoiCandidates ready for publication.
 @return RoiCandidates with numCandidates set and candidates[0] = rank-1.
 @param candidates  Unsorted candidate list (taken by value; sorted in-place).
 @param numRows     Number of rows in the image (defines the image center's row coordinate).
 @param numCols     Number of columns in the image (defines the image center's column coordinate).
*/
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
