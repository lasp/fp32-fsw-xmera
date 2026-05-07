#include "architecture/testUtilities/eigenFuzzDomains.hpp"
#include "forceTorqueThrForceMappingTestHelpers.hpp"
#include <fuzztest/fuzztest.h>

FUZZ_TEST(ForceTorqueThrForceMappingFuzz, runRegressionCase)
    .WithDomains(fuzztest::InRange<std::uint32_t>(1U, MAX_EFF_CNT),
                 fuzztest::VectorOf(xmera::fuzz::Vector3fInRange(-5.0F, 5.0F)).WithSize(MAX_EFF_CNT),
                 fuzztest::VectorOf(xmera::fuzz::Vector3fInRange(-1.0F, 1.0F)).WithSize(MAX_EFF_CNT),
                 xmera::fuzz::Vector3fInRange(-2.0F, 2.0F),
                 xmera::fuzz::Vector3fInRange(-10.0F, 10.0F),
                 xmera::fuzz::Vector3fInRange(-1e3F, 1e3F));

// ---------------------------------------------------------------------------
// Property fuzz tests
// ---------------------------------------------------------------------------

FUZZ_TEST(ForceTorqueThrForceMappingPropertyFuzz, propertyNonNegativeForces)
    .WithDomains(fuzztest::InRange<std::uint32_t>(1U, MAX_EFF_CNT),
                 fuzztest::VectorOf(xmera::fuzz::Vector3fInRange(-5.0F, 5.0F)).WithSize(MAX_EFF_CNT),
                 fuzztest::VectorOf(xmera::fuzz::Vector3fInRange(-1.0F, 1.0F)).WithSize(MAX_EFF_CNT),
                 xmera::fuzz::Vector3fInRange(-2.0F, 2.0F),
                 xmera::fuzz::Vector3fInRange(-10.0F, 10.0F),
                 xmera::fuzz::Vector3fInRange(-1e3F, 1e3F));

FUZZ_TEST(ForceTorqueThrForceMappingPropertyFuzz, propertyMinimumIsZero)
    .WithDomains(fuzztest::InRange<std::uint32_t>(1U, MAX_EFF_CNT),
                 fuzztest::VectorOf(xmera::fuzz::Vector3fInRange(-5.0F, 5.0F)).WithSize(MAX_EFF_CNT),
                 fuzztest::VectorOf(xmera::fuzz::Vector3fInRange(-1.0F, 1.0F)).WithSize(MAX_EFF_CNT),
                 xmera::fuzz::Vector3fInRange(-2.0F, 2.0F),
                 xmera::fuzz::Vector3fInRange(-10.0F, 10.0F),
                 xmera::fuzz::Vector3fInRange(-1e3F, 1e3F));

FUZZ_TEST(ForceTorqueThrForceMappingPropertyFuzz, propertyPaddingIsZero)
    .WithDomains(fuzztest::InRange<std::uint32_t>(1U, MAX_EFF_CNT),
                 fuzztest::VectorOf(xmera::fuzz::Vector3fInRange(-5.0F, 5.0F)).WithSize(MAX_EFF_CNT),
                 fuzztest::VectorOf(xmera::fuzz::Vector3fInRange(-1.0F, 1.0F)).WithSize(MAX_EFF_CNT),
                 xmera::fuzz::Vector3fInRange(-2.0F, 2.0F),
                 xmera::fuzz::Vector3fInRange(-10.0F, 10.0F),
                 xmera::fuzz::Vector3fInRange(-1e3F, 1e3F));

// Scale-invariance is mathematically exact for any rank, but fp32 noise scales with ||pinv|| * ||ft||.
// Truncated-SVD bounds ||pinv|| <= 1/(sigma_max * eps * max(m,n)) ~ 2.5e5/sigma_max, so the noise
// floor is finite — but inputs near the truncation threshold (where sigma_min_kept barely exceeds
// the cutoff) still drive ||pinv|| toward that bound. Empirically, numThrusters >= 6 keeps the
// kept singular values comfortably above the truncation tol for almost all fuzz inputs in the
// configured ranges. Command magnitude and scale factor stay capped so that min-shift cancellation
// (subtraction of two values each with eps*||pinv||*||ft|| absolute precision) stays inside the
// test's atol=1e-4 budget.
FUZZ_TEST(ForceTorqueThrForceMappingPropertyFuzz, propertyScaleInvariance)
    .WithDomains(fuzztest::InRange<std::uint32_t>(6U, MAX_EFF_CNT),
                 fuzztest::VectorOf(xmera::fuzz::Vector3fInRange(-5.0F, 5.0F)).WithSize(MAX_EFF_CNT),
                 fuzztest::VectorOf(xmera::fuzz::Vector3fInRange(-1.0F, 1.0F)).WithSize(MAX_EFF_CNT),
                 xmera::fuzz::Vector3fInRange(-2.0F, 2.0F),
                 xmera::fuzz::Vector3fInRange(-10.0F, 10.0F),
                 xmera::fuzz::Vector3fInRange(-10.0F, 10.0F),
                 fuzztest::InRange(0.1F, 10.0F));

FUZZ_TEST(ForceTorqueThrForceMappingPropertyFuzz, propertyStateless)
    .WithDomains(fuzztest::InRange<std::uint32_t>(1U, MAX_EFF_CNT),
                 fuzztest::VectorOf(xmera::fuzz::Vector3fInRange(-5.0F, 5.0F)).WithSize(MAX_EFF_CNT),
                 fuzztest::VectorOf(xmera::fuzz::Vector3fInRange(-1.0F, 1.0F)).WithSize(MAX_EFF_CNT),
                 xmera::fuzz::Vector3fInRange(-2.0F, 2.0F),
                 xmera::fuzz::Vector3fInRange(-10.0F, 10.0F),
                 xmera::fuzz::Vector3fInRange(-1e3F, 1e3F));

FUZZ_TEST(ForceTorqueThrForceMappingPropertyFuzz, propertyFiniteOutput)
    .WithDomains(fuzztest::InRange<std::uint32_t>(1U, MAX_EFF_CNT),
                 fuzztest::VectorOf(xmera::fuzz::Vector3fInRange(-5.0F, 5.0F)).WithSize(MAX_EFF_CNT),
                 fuzztest::VectorOf(xmera::fuzz::Vector3fInRange(-1.0F, 1.0F)).WithSize(MAX_EFF_CNT),
                 xmera::fuzz::Vector3fInRange(-2.0F, 2.0F),
                 xmera::fuzz::Vector3fInRange(-10.0F, 10.0F),
                 xmera::fuzz::Vector3fInRange(-1e3F, 1e3F));

// Reachable-command reproduction on a fixed balanced layout. The layout (rcsPositions1 /
// rcsDirections1, verified balanced with sum(g)=0 and sum(r×g)=0) stays fixed so DG·1 = 0 and the
// min-shift is FT-preserving. CoM and per-thruster forces fuzz; cmd = DG·x_test is constructed
// inside the helper so it's automatically in the row space. The check is the direct
// achieved == cmd, since DG·1 = 0 leaves no min-shift correction to absorb.
FUZZ_TEST(ForceTorqueThrForceMappingPropertyFuzz, propertyAchievesCommandForBalancedLayout)
    .WithDomains(xmera::fuzz::Vector3fInRange(-2.0F, 2.0F),
                 xmera::fuzz::EigenVectorOf<float, 8>(fuzztest::InRange(0.0F, 10.0F)));

FUZZ_TEST(ForceTorqueThrForceMappingPropertyFuzz, propertyOutputMagnitudeBounded)
    .WithDomains(fuzztest::InRange<std::uint32_t>(1U, MAX_EFF_CNT),
                 fuzztest::VectorOf(xmera::fuzz::Vector3fInRange(-5.0F, 5.0F)).WithSize(MAX_EFF_CNT),
                 fuzztest::VectorOf(xmera::fuzz::Vector3fInRange(-1.0F, 1.0F)).WithSize(MAX_EFF_CNT),
                 xmera::fuzz::Vector3fInRange(-2.0F, 2.0F),
                 xmera::fuzz::Vector3fInRange(-10.0F, 10.0F),
                 xmera::fuzz::Vector3fInRange(-1e3F, 1e3F));
