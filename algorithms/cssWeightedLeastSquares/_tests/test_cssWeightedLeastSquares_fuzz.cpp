#include "cssWeightedLeastSquaresTestHelpers.hpp"
#include <fuzztest/fuzztest.h>
#include <utilities/testUtilities/eigenFuzzDomains.hpp>

namespace {

// Boresights are drawn over the unit cube and normalized by the helper, which rejects the ones too short to
// point anywhere. Biases reach past one so a miscalibrated sensor is covered, and zero is included because
// that is how a sensor is disabled.
auto constellationDomain() {
    return fuzztest::StructOf<ConstellationInputs>(
        fuzztest::InRange(0U, static_cast<uint32_t>(kMaxNumCssSensors) + 1U),
        fuzztest::VectorOf(fuzztest::InRange(-1.0F, 1.0F)).WithSize(static_cast<size_t>(kMaxNumCssSensors) * 3U),
        fuzztest::VectorOf(fuzztest::InRange(0.0F, 2.0F)).WithSize(static_cast<size_t>(kMaxNumCssSensors)),
        fuzztest::Arbitrary<bool>(),
        fuzztest::InRange(-0.1F, 1.1F),
        fuzztest::InRange(-0.1F, 10.0F));
}

// A coarse sun sensor reports a cosine, so readings stay in [0, 1]. The threshold domain reaches outside
// that range, and the count domain outside its bounds, so the configuration rejections are exercised too.
auto readingsDomain() {
    return fuzztest::VectorOf(fuzztest::InRange(0.0F, 1.0F)).WithSize(static_cast<size_t>(kMaxNumCssSensors));
}

// The readings a working sensor reports, mixed with the full range of values a broken one can report. The
// mixture keeps the fit covered as well as the rejection.
auto brokenReadingsDomain() {
    return fuzztest::VectorOf(fuzztest::OneOf(fuzztest::InRange(-0.1F, 1.1F), fuzztest::Arbitrary<float>()))
        .WithSize(static_cast<size_t>(kMaxNumCssSensors));
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
    .WithDomains(constellationDomain(), brokenReadingsDomain());

FUZZ_TEST(CssWeightedLeastSquaresPropertyFuzz, propertyHeadingIsUnitOrZero)
    .WithDomains(constellationDomain(), brokenReadingsDomain());

FUZZ_TEST(CssWeightedLeastSquaresPropertyFuzz, propertyResidualsPaddedWithZeros)
    .WithDomains(constellationDomain(), brokenReadingsDomain());

FUZZ_TEST(CssWeightedLeastSquaresPropertyFuzz, propertyDisabledSensorIgnored)
    .WithDomains(constellationDomain(),
                 readingsDomain(),
                 fuzztest::InRange(0U, static_cast<uint32_t>(kMaxNumCssSensors) - 1U));

FUZZ_TEST(CssWeightedLeastSquaresPropertyFuzz, propertyRotationEquivariance)
    .WithDomains(constellationDomain(), readingsDomain(), xmera::fuzz::Vector3fInRange(-3.2F, 3.2F));

FUZZ_TEST(CssWeightedLeastSquaresPropertyFuzz, propertyRateOrthogonalToHeading)
    .WithDomains(constellationDomain(), readingsDomain(), readingsDomain());
