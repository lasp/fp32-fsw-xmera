#include "dvGuidanceTestHelpers.hpp"
#include "utilities/testUtilities/eigenFuzzDomains.hpp"

#include <fuzztest/fuzztest.h>

namespace {

/// Fuzzes the configuration inputs across the full domain. The fuzzed preferred direction rPref_N is
/// normalized to the unit rHatPref_N that create() requires. Configurations that create() rejects
/// (zero-norm cmdDv_N, rHatPref_N (anti)parallel to it) are skipped; the rejection itself is covered by
/// explicit edge tests. testDvGuidanceRegression compares against the double-precision reference at
/// 1e-4 tolerance.
void fuzzDvGuidance(const Eigen::Vector3f& cmdDv_N, const Eigen::Vector3f& rPref_N) {
    const Eigen::Vector3f rHatPref_N = rPref_N.stableNormalized();
    if (!DvGuidanceConfig::isValidCmdDv(cmdDv_N) || !DvGuidanceConfig::isValidRHatPref(cmdDv_N, rHatPref_N)) {
        return;
    }
    testDvGuidanceRegression(cmdDv_N, rHatPref_N);
}

void fuzzPropertyOutputIsFinite(const Eigen::Vector3f& cmdDv_N, const Eigen::Vector3f& rPref_N) {
    const Eigen::Vector3f rHatPref_N = rPref_N.stableNormalized();
    if (!DvGuidanceConfig::isValidCmdDv(cmdDv_N) || !DvGuidanceConfig::isValidRHatPref(cmdDv_N, rHatPref_N)) {
        return;
    }
    propertyOutputIsFinite(cmdDv_N, rHatPref_N);
}

void fuzzPropertySigmaNormBounded(const Eigen::Vector3f& cmdDv_N, const Eigen::Vector3f& rPref_N) {
    const Eigen::Vector3f rHatPref_N = rPref_N.stableNormalized();
    if (!DvGuidanceConfig::isValidCmdDv(cmdDv_N) || !DvGuidanceConfig::isValidRHatPref(cmdDv_N, rHatPref_N)) {
        return;
    }
    propertySigmaNormBounded(cmdDv_N, rHatPref_N);
}

void fuzzPropertyThirdAxisAlignedWithDeltaV(const Eigen::Vector3f& cmdDv_N, const Eigen::Vector3f& rPref_N) {
    const Eigen::Vector3f rHatPref_N = rPref_N.stableNormalized();
    if (!DvGuidanceConfig::isValidCmdDv(cmdDv_N) || !DvGuidanceConfig::isValidRHatPref(cmdDv_N, rHatPref_N)) {
        return;
    }
    propertyThirdAxisAlignedWithDeltaV(cmdDv_N, rHatPref_N);
}

void fuzzPropertySecondAxisTowardRHatPref(const Eigen::Vector3f& cmdDv_N, const Eigen::Vector3f& rPref_N) {
    const Eigen::Vector3f rHatPref_N = rPref_N.stableNormalized();
    if (!DvGuidanceConfig::isValidCmdDv(cmdDv_N) || !DvGuidanceConfig::isValidRHatPref(cmdDv_N, rHatPref_N)) {
        return;
    }
    propertySecondAxisTowardRHatPref(cmdDv_N, rHatPref_N);
}

}  // namespace

FUZZ_TEST(DvGuidancePropertyFuzz, fuzzPropertyOutputIsFinite)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1.0e4F, 1.0e4F), xmera::fuzz::Vector3fInRange(-1.0F, 1.0F));

FUZZ_TEST(DvGuidancePropertyFuzz, fuzzPropertySigmaNormBounded)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1.0e4F, 1.0e4F), xmera::fuzz::Vector3fInRange(-1.0F, 1.0F));

FUZZ_TEST(DvGuidancePropertyFuzz, fuzzPropertyThirdAxisAlignedWithDeltaV)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1.0e4F, 1.0e4F), xmera::fuzz::Vector3fInRange(-1.0F, 1.0F));

FUZZ_TEST(DvGuidancePropertyFuzz, fuzzPropertySecondAxisTowardRHatPref)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1.0e4F, 1.0e4F), xmera::fuzz::Vector3fInRange(-1.0F, 1.0F));

FUZZ_TEST(DvGuidanceFuzz, fuzzDvGuidance)
    .WithDomains(
        // cmdDv_N [m/s]: realistic delta-V envelope, unfiltered so near-/zero-norm inputs reach the
        // config validation.
        xmera::fuzz::Vector3fInRange(-1.0e4F, 1.0e4F),
        // rPref_N: normalized to rHatPref_N before use. Unfiltered so (anti)parallel-to-cmdDv_N directions
        // reach the config validation.
        xmera::fuzz::Vector3fInRange(-1.0F, 1.0F));
