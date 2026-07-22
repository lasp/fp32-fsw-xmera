#include "triadTestHelpers.hpp"
#include "utilities/testUtilities/eigenFuzzDomains.hpp"
#include <fuzztest/fuzztest.h>

// ---------------------------------------------------------------------------
// Regression fuzz: random configs and reference inputs must agree with the
// independent reference implementation across multiple steps.
// ---------------------------------------------------------------------------
void fuzzTriadRegression(const Eigen::Vector3f& rHat_SB_N,
                         const Eigen::Vector3f& thrustHat_B,
                         const Eigen::Vector3f& sadaHat_B,
                         const Eigen::Vector3f& thrustReqHat_N,
                         const N3Axis n3Axis) {
    if (sadaHat_B.stableNorm() == 0.0F || thrustReqHat_N.stableNorm() == 0.0F) {
        return;
    }

    testTriadRegression(rHat_SB_N.stableNormalized(),
                        thrustHat_B.stableNormalized(),
                        sadaHat_B.stableNormalized(),
                        thrustReqHat_N.stableNormalized(),
                        n3Axis);
}

FUZZ_TEST(TriadAlgorithmFuzz, fuzzTriadRegression)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),                                // rHat_SB_N
                 xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),                                // thrustHat_B
                 xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),                                // sadaAxisHat_B
                 xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),                                // thrustReqHat_N
                 fuzztest::ElementOf<N3Axis>({N3Axis::plusZHat_N, N3Axis::minusZHat_N}));  // n3Axis

// ---------------------------------------------------------------------------
// Property fuzz tests
// ---------------------------------------------------------------------------
void fuzzPropertyOutputIsFinite(const Eigen::Vector3f& rHat_SB_N,
                                const Eigen::Vector3f& thrustHat_B,
                                const Eigen::Vector3f& sadaHat_B,
                                const Eigen::Vector3f& thrustReqHat_N,
                                const N3Axis n3Axis) {
    if (sadaHat_B.stableNorm() == 0.0F || thrustReqHat_N.stableNorm() == 0.0F) {
        return;
    }

    propertyOutputIsFinite(rHat_SB_N.stableNormalized(),
                           thrustHat_B.stableNormalized(),
                           sadaHat_B.stableNormalized(),
                           thrustReqHat_N.stableNormalized(),
                           n3Axis);
}

FUZZ_TEST(TriadPropertyFuzz, fuzzPropertyOutputIsFinite)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),                                // rHat_SB_N
                 xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),                                // thrustHat_B
                 xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),                                // sadaAxisHat_B
                 xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),                                // thrustReqHat_N
                 fuzztest::ElementOf<N3Axis>({N3Axis::plusZHat_N, N3Axis::minusZHat_N}));  // n3Axis

void fuzzPropertyThrustBodyHeadingAlignedToThrustInertialHeading(const Eigen::Vector3f& rHat_SB_N,
                                                                 const Eigen::Vector3f& thrustHat_B,
                                                                 const Eigen::Vector3f& sadaHat_B,
                                                                 const Eigen::Vector3f& thrustReqHat_N,
                                                                 const N3Axis n3Axis) {
    const Eigen::Vector3f rHatUnit_SB_N = rHat_SB_N.stableNormalized();
    const Eigen::Vector3f thrustHatUnit_B = thrustHat_B.stableNormalized();
    const Eigen::Vector3f sadaHatUnit_B = sadaHat_B.stableNormalized();
    const float sadaAxisToThrustAngle = safeAcosf(fabsf(sadaHatUnit_B.dot(thrustHatUnit_B)));

    // Skip edge cases where the algorithm returns identity attitude
    if (sadaAxisToThrustAngle < kParallelThresholdRad || thrustHatUnit_B.stableNorm() == 0.0F ||
        rHatUnit_SB_N.stableNorm() == 0.0F || sadaHat_B.stableNorm() == 0.0F || thrustReqHat_N.stableNorm() == 0.0F) {
        return;
    }

    // Skip the degenerate fallback: when the sun direction is parallel to the thrust reference (so the algorithm
    // crosses with the configured inertial z-axis) and that z-axis is itself parallel to the thrust reference, the
    // algorithm returns identity attitude (no alignment guarantee)
    const Eigen::Vector3f thrustReqHatUnit_N = thrustReqHat_N.stableNormalized();
    const float sunToThrustRefAngle = safeAcosf(fabsf(rHatUnit_SB_N.dot(thrustReqHatUnit_N)));
    const float n3HatSign = (n3Axis == N3Axis::plusZHat_N) ? 1.0F : -1.0F;
    const Eigen::Vector3f n3Hat_N = (n3HatSign * Eigen::Vector3f::UnitZ()).normalized();
    const float zToThrustRefAngle = safeAcosf(fabsf(n3Hat_N.dot(thrustReqHatUnit_N)));
    if (sunToThrustRefAngle < kParallelThresholdRad && zToThrustRefAngle < kParallelThresholdRad) {
        return;
    }

    propertyThrustBodyHeadingAlignedToThrustInertialHeading(
        rHatUnit_SB_N, thrustHatUnit_B, sadaHatUnit_B, thrustReqHat_N.stableNormalized(), n3Axis);
}

FUZZ_TEST(TriadPropertyFuzz, fuzzPropertyThrustBodyHeadingAlignedToThrustInertialHeading)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),                                // rHat_SB_N
                 xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),                                // thrustHat_B
                 xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),                                // sadaAxisHat_B
                 xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),                                // thrustReqHat_N
                 fuzztest::ElementOf<N3Axis>({N3Axis::plusZHat_N, N3Axis::minusZHat_N}));  // n3Axis

void fuzzPropertySigmaNormBounded(const Eigen::Vector3f& rHat_SB_N,
                                  const Eigen::Vector3f& thrustHat_B,
                                  const Eigen::Vector3f& sadaHat_B,
                                  const Eigen::Vector3f& thrustReqHat_N,
                                  const N3Axis n3Axis) {
    if (sadaHat_B.stableNorm() == 0.0F || thrustReqHat_N.stableNorm() == 0.0F) {
        return;
    }

    propertySigmaNormBounded(rHat_SB_N.stableNormalized(),
                             thrustHat_B.stableNormalized(),
                             sadaHat_B.stableNormalized(),
                             thrustReqHat_N.stableNormalized(),
                             n3Axis);
}

FUZZ_TEST(TriadPropertyFuzz, fuzzPropertySigmaNormBounded)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),                                // rHat_SB_N
                 xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),                                // thrustHat_B
                 xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),                                // sadaAxisHat_B
                 xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),                                // thrustReqHat_N
                 fuzztest::ElementOf<N3Axis>({N3Axis::plusZHat_N, N3Axis::minusZHat_N}));  // n3Axis

void fuzzPropertySolarArraySunOffsetBoundedByBodyThrustOffset(const Eigen::Vector3f& rHat_SB_N,
                                                              const Eigen::Vector3f& thrustHat_B,
                                                              const Eigen::Vector3f& sadaHat_B,
                                                              const Eigen::Vector3f& thrustReqHat_N,
                                                              const N3Axis n3Axis) {
    if (sadaHat_B.stableNorm() == 0.0F || thrustReqHat_N.stableNorm() == 0.0F) {
        return;
    }

    // Skip two sadaHat_B & thrustHat_B alignment cases:
    // (1) Edge case where sadaHat_B & thrustHat_B are aligned (zero MRP returned, thrust alignment not guaranteed)
    // (2) sadaHat_B & thrustHat_B are near orthogonal: thrust offset angle is zero and corresponding sada-Sun offset is
    // zero (Here sada-sun offset = body thrust offset, but this check is strictly for cases where the sada-Sun offset <
    // body thrust offset)
    const Eigen::Vector3f thrustHatUnit_B = thrustHat_B.stableNormalized();
    const Eigen::Vector3f sadaHatUnit_B = sadaHat_B.stableNormalized();
    const float sadaToThrustAngle = safeAcosf(fabsf(sadaHatUnit_B.dot(thrustHatUnit_B)));
    if (sadaToThrustAngle < kParallelThresholdRad ||
        sadaToThrustAngle > 0.5F * std::numbers::pi_v<float> - kParallelThresholdRad) {
        return;
    }

    // Skip the fallback case where the zero MRP is returned (thrust alignment not guaranteed)
    const Eigen::Vector3f rHatUnit_SB_N = rHat_SB_N.stableNormalized();
    const Eigen::Vector3f thrustReqHatUnit_N = thrustReqHat_N.stableNormalized();
    const float sunToThrustRefAngle = safeAcosf(fabsf(rHatUnit_SB_N.dot(thrustReqHatUnit_N)));
    if (sunToThrustRefAngle < kParallelThresholdRad) {
        return;
    }

    propertySolarArraySunOffsetBoundedByBodyThrustOffset(
        rHatUnit_SB_N, thrustHatUnit_B, sadaHatUnit_B, thrustReqHatUnit_N, n3Axis);
}

FUZZ_TEST(TriadPropertyFuzz, fuzzPropertySolarArraySunOffsetBoundedByBodyThrustOffset)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),                                // rHat_SB_N
                 xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),                                // thrustHat_B
                 xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),                                // sadaAxisHat_B
                 xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),                                // thrustReqHat_N
                 fuzztest::ElementOf<N3Axis>({N3Axis::plusZHat_N, N3Axis::minusZHat_N}));  // n3Axis
