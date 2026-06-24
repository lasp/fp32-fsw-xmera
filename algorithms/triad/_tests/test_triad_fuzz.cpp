#include "triadTestHelpers.hpp"
#include "utilities/testUtilities/eigenFuzzDomains.hpp"
#include <fuzztest/fuzztest.h>

// ---------------------------------------------------------------------------
// Regression fuzz: random configs and reference inputs must agree with the
// independent reference implementation across multiple steps.
// ---------------------------------------------------------------------------
void fuzzTriadRegression(const Eigen::Vector3f& sigma_BN,
                         const Eigen::Vector3f& rHat_SB_B,
                         const Eigen::Vector3f& thrustHat_B,
                         const Eigen::Vector3f& sadaHat_B,
                         const Eigen::Vector3f& thrustReqHat_N,
                         const float signOfZHat_N) {
    if (signOfZHat_N == 0.0F || sadaHat_B.stableNorm() == 0.0F || thrustReqHat_N.stableNorm() == 0.0F) {
        return;
    }

    testTriadRegression(sigma_BN,
        rHat_SB_B.stableNormalized(),
        thrustHat_B.stableNormalized(),
        sadaHat_B.stableNormalized(),
        thrustReqHat_N.stableNormalized(),
        signOfZHat_N);
}

FUZZ_TEST(TriadAlgorithmFuzz, fuzzTriadRegression)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),  // sigma_BN
                 xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),  // rHat_SB_B
                 xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),  // thrustHat_B
                 xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),  // sadaAxisHat_B
                 xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),  // thrustReqHat_N
                 fuzztest::InRange(-1e6F, 1e6F));  // signOfZHat_N

// ---------------------------------------------------------------------------
// Property fuzz tests
// ---------------------------------------------------------------------------
void fuzzPropertyOutputIsFinite(const Eigen::Vector3f& sigma_BN,
                                const Eigen::Vector3f& rHat_SB_B,
                                const Eigen::Vector3f& thrustHat_B,
                                const Eigen::Vector3f& sadaHat_B,
                                const Eigen::Vector3f& thrustReqHat_N,
                                const float signOfZHat_N) {
    if (signOfZHat_N == 0.0F || sadaHat_B.stableNorm() == 0.0F || thrustReqHat_N.stableNorm() == 0.0F) {
        return;
    }

    propertyOutputIsFinite(sigma_BN,
                           rHat_SB_B.stableNormalized(),
                           thrustHat_B.stableNormalized(),
                           sadaHat_B.stableNormalized(),
                           thrustReqHat_N.stableNormalized(),
                           signOfZHat_N);
}

FUZZ_TEST(TriadPropertyFuzz, fuzzPropertyOutputIsFinite)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),  // sigma_BN
                 xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),  // rHat_SB_B
                 xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),  // thrustHat_B
                 xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),  // sadaAxisHat_B
                 xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),  // thrustReqHat_N
                 fuzztest::InRange(-1e6F, 1e6F));  // signOfZHat_N

void fuzzPropertyBodyHeadingAlignedToInertialHeading(const Eigen::Vector3f& sigma_BN,
                                                     const Eigen::Vector3f& rHat_SB_B,
                                                     const Eigen::Vector3f& thrustHat_B,
                                                     const Eigen::Vector3f& sadaHat_B,
                                                     const Eigen::Vector3f& thrustReqHat_N,
                                                     const float signOfZHat_N) {
    const Eigen::Vector3f rHatUnit_SB_B = rHat_SB_B.stableNormalized();
    const Eigen::Vector3f thrustHatUnit_B = thrustHat_B.stableNormalized();
    const Eigen::Vector3f sadaHatUnit_B = sadaHat_B.stableNormalized();
    const float sadaAxisToThrustAngle = safeAcosf(fabsf(sadaHatUnit_B.dot(thrustHatUnit_B)));

    // Skip configs the algorithm answers with the current attitude (no alignment guarantee)
    if (signOfZHat_N == 0.0F || sadaAxisToThrustAngle < kParallelThresholdRad || thrustHatUnit_B.stableNorm() == 0.0F ||
        rHatUnit_SB_B.stableNorm() == 0.0F || sadaHat_B.stableNorm() == 0.0F || thrustReqHat_N.stableNorm() == 0.0F) {
        return;
    }

    // Skip the degenerate fallback: when the sun direction is parallel to the thrust reference (so the algorithm
    // crosses with the configured inertial z-axis) and that z-axis is itself parallel to the thrust reference, the
    // algorithm returns the current attitude (no alignment guarantee)
    const Eigen::Vector3f thrustReqHatUnit_N = thrustReqHat_N.stableNormalized();
    const Eigen::Matrix3f dcm_BN = mrpToDcm(sigma_BN);
    const Eigen::Vector3f rHat_SB_N = (dcm_BN.transpose() * rHatUnit_SB_B).normalized();
    const float sunToThrustRefAngle = safeAcosf(fabsf(rHat_SB_N.dot(thrustReqHatUnit_N)));
    const Eigen::Vector3f zHat_N = (copysignf(1.0F, signOfZHat_N) * Eigen::Vector3f::UnitZ()).normalized();
    const float zToThrustRefAngle = safeAcosf(fabsf(zHat_N.dot(thrustReqHatUnit_N)));
    if (sunToThrustRefAngle < kParallelThresholdRad && zToThrustRefAngle < kParallelThresholdRad) {
        return;
    }

    propertyBodyHeadingAlignedToInertialHeading(
        sigma_BN, rHatUnit_SB_B, thrustHatUnit_B, sadaHatUnit_B, thrustReqHat_N.stableNormalized(), signOfZHat_N);
}

FUZZ_TEST(TriadPropertyFuzz, fuzzPropertyBodyHeadingAlignedToInertialHeading)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),  // sigma_BN
                 xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),  // rHat_SB_B
                 xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),  // thrustHat_B
                 xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),  // sadaAxisHat_B
                 xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),  // thrustReqHat_N
                 fuzztest::InRange(-1e6F, 1e6F));  // signOfZHat_N

void fuzzPropertySigmaNormBounded(const Eigen::Vector3f& sigma_BN,
                                  const Eigen::Vector3f& rHat_SB_B,
                                  const Eigen::Vector3f& thrustHat_B,
                                  const Eigen::Vector3f& sadaHat_B,
                                  const Eigen::Vector3f& thrustReqHat_N,
                                  const float signOfZHat_N) {
    if (signOfZHat_N == 0.0F || sadaHat_B.stableNorm() == 0.0F || thrustReqHat_N.stableNorm() == 0.0F) {
        return;
    }

    propertySigmaNormBounded(sigma_BN,
                             rHat_SB_B.stableNormalized(),
                             thrustHat_B.stableNormalized(),
                             sadaHat_B.stableNormalized(),
                             thrustReqHat_N.stableNormalized(),
                             signOfZHat_N);
}

FUZZ_TEST(TriadPropertyFuzz, fuzzPropertySigmaNormBounded)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),  // sigma_BN
                 xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),  // rHat_SB_B
                 xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),  // thrustHat_B
                 xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),  // sadaAxisHat_B
                 xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),  // thrustReqHat_N
                 fuzztest::InRange(-1e6F, 1e6F));  // signOfZHat_N
