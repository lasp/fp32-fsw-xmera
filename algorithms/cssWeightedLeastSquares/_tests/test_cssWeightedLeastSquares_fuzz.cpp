#include "cssWeightedLeastSquaresTestHelpers.hpp"
#include <fuzztest/fuzztest.h>
#include <utilities/testUtilities/eigenFuzzDomains.hpp>

namespace {

// Boresights are drawn over the unit cube and normalized by the helper, which rejects the ones too short to
// point anywhere. Every combination of available and unavailable sensors is covered, the all-unavailable
// one included, which the configuration must reject.
auto constellationDomain() {
    return fuzztest::StructOf<ConstellationInputs>(
        fuzztest::VectorOf(fuzztest::InRange(-1.0F, 1.0F)).WithSize(static_cast<size_t>(kMaxNumCssSensors) * 3U),
        fuzztest::VectorOf(fuzztest::Arbitrary<bool>()).WithSize(static_cast<size_t>(kMaxNumCssSensors)),
        fuzztest::Arbitrary<bool>(),
        fuzztest::InRange(-0.1F, 1.1F),
        fuzztest::InRange(-0.1F, 10.0F));
}

// The readings the sensor module publishes. It clamps its output to the range a cosine occupies, so the
// estimator applies no bound of its own. The margin either side covers a reading at the threshold and one
// at full scale.
auto readingsDomain() {
    return fuzztest::VectorOf(fuzztest::InRange(-0.1F, 1.1F)).WithSize(static_cast<size_t>(kMaxNumCssSensors));
}

}  // namespace

// ---------------------------------------------------------------------------
// Regression fuzz test — update() against the independent fp64 reference.
// ---------------------------------------------------------------------------

FUZZ_TEST(CssWeightedLeastSquaresFuzz, runRegressionCase).WithDomains(constellationDomain(), readingsDomain());

// ---------------------------------------------------------------------------
// Property fuzz tests
// ---------------------------------------------------------------------------

// The finiteness property is the one that has to face a broken sensor, so it takes the wider domain.
FUZZ_TEST(CssWeightedLeastSquaresPropertyFuzz, propertyOutputIsFinite)
    .WithDomains(constellationDomain(), readingsDomain());

FUZZ_TEST(CssWeightedLeastSquaresPropertyFuzz, propertyHeadingIsUnitOrZero)
    .WithDomains(constellationDomain(), readingsDomain());

FUZZ_TEST(CssWeightedLeastSquaresPropertyFuzz, propertyResidualsPaddedWithZeros)
    .WithDomains(constellationDomain(), readingsDomain());

FUZZ_TEST(CssWeightedLeastSquaresPropertyFuzz, propertyDisabledSensorIgnored)
    .WithDomains(constellationDomain(),
                 readingsDomain(),
                 fuzztest::InRange(0U, static_cast<uint32_t>(kMaxNumCssSensors) - 1U));

FUZZ_TEST(CssWeightedLeastSquaresPropertyFuzz, propertyRotationEquivariance)
    .WithDomains(constellationDomain(), readingsDomain(), xmera::fuzz::Vector3fInRange(-3.2F, 3.2F));

FUZZ_TEST(CssWeightedLeastSquaresPropertyFuzz, propertyRateOrthogonalToHeading)
    .WithDomains(constellationDomain(), readingsDomain(), readingsDomain());
