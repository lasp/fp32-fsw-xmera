#include "axisToGimbalAnglesTestHelpers.hpp"
#include "utilities/testUtilities/eigenFuzzDomains.hpp"
#include <fuzztest/fuzztest.h>
#include <numbers>

namespace {
// The travel limit holds both angles inside thetaMax. A pair of angles that builds a deflection beyond the
// travel is therefore not a value that the module can give again. A pair inside 45 degrees builds a deflection
// of at most 55 degrees, which stays inside the default travel that the helpers use.
constexpr float kMaxAngle = 45.0F * (std::numbers::pi_v<float> / 180.0F);

// The travel range covers a small gimbal and one that is near the 90 degree limit of the two angles.
constexpr float kMinThetaMax = 5.0F * (std::numbers::pi_v<float> / 180.0F);
constexpr float kMinThetaMaxForAngles = 60.0F * (std::numbers::pi_v<float> / 180.0F);
constexpr float kMaxThetaMax = 85.0F * (std::numbers::pi_v<float> / 180.0F);

// Direction components stay near unit length. This keeps the directions in all parts of the sphere and includes
// the zero vector. The length of the direction has no effect on the two angles, and propertyLengthHasNoEffect
// examines that property across the full float range.
constexpr float kDirectionLimit = 1.0F;

// The mounting range covers the principal MRP set and more, which create() changes to the shadow set.
constexpr float kMrpLimit = 2.0F;
}  // namespace

// ---------------------------------------------------------------------------
// Regression fuzz tests
// ---------------------------------------------------------------------------

// The direction range includes the origin and all deflections, thus the fuzz test also examines the requests
// that the travel limit must pull back onto the cone.
FUZZ_TEST(AxisToGimbalAnglesFuzz, regressionTestAxisToGimbalAngles)
    .WithDomains(xmera::fuzz::Vector3fInRange(-kMrpLimit, kMrpLimit),              // sigma_MB (MRP)
                 xmera::fuzz::Vector3fInRange(-kDirectionLimit, kDirectionLimit),  // direction, body frame
                 fuzztest::InRange(kMinThetaMax, kMaxThetaMax));                   // thetaMax [rad]

// The module can only give a pair of angles again when the travel does not limit the direction that the pair
// builds. A pair inside 45 degrees builds a deflection of at most 55 degrees, thus the travel starts above it.
FUZZ_TEST(AxisToGimbalAnglesFuzz, regressionTestAxisToGimbalAnglesFromAngles)
    .WithDomains(xmera::fuzz::Vector3fInRange(-kMrpLimit, kMrpLimit),      // sigma_MB (MRP)
                 fuzztest::InRange(-kMaxAngle, kMaxAngle),                 // angle1 [rad]
                 fuzztest::InRange(-kMaxAngle, kMaxAngle),                 // angle2 [rad]
                 fuzztest::InRange(kMinThetaMaxForAngles, kMaxThetaMax));  // thetaMax [rad]

// ---------------------------------------------------------------------------
// Property fuzz tests
// ---------------------------------------------------------------------------

FUZZ_TEST(AxisToGimbalAnglesPropertyFuzz, propertyOutputIsUsable)
    .WithDomains(xmera::fuzz::Vector3fInRange(-kMrpLimit, kMrpLimit),
                 xmera::fuzz::Vector3fInRange(-kDirectionLimit, kDirectionLimit),
                 fuzztest::InRange(kMinThetaMax, kMaxThetaMax));

FUZZ_TEST(AxisToGimbalAnglesPropertyFuzz, propertyDirectionRecovered)
    .WithDomains(xmera::fuzz::Vector3fInRange(-kMrpLimit, kMrpLimit),
                 xmera::fuzz::Vector3fInRange(-kDirectionLimit, kDirectionLimit),
                 fuzztest::InRange(kMinThetaMax, kMaxThetaMax));

// The module makes the input direction a unit vector, thus the length has no effect. The range below keeps each
// scaled component in the normal float range, where the scaling keeps the direction.
FUZZ_TEST(AxisToGimbalAnglesPropertyFuzz, propertyLengthHasNoEffect)
    .WithDomains(xmera::fuzz::Vector3fInRange(-kMrpLimit, kMrpLimit),
                 xmera::fuzz::Vector3fInRange(-kDirectionLimit, kDirectionLimit),
                 fuzztest::InRange(1e-3F, 1e3F),
                 fuzztest::InRange(kMinThetaMax, kMaxThetaMax));
