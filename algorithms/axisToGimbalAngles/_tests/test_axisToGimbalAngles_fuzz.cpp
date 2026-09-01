#include "axisToGimbalAnglesTestHelpers.hpp"
#include "utilities/testUtilities/eigenFuzzDomains.hpp"
#include <fuzztest/fuzztest.h>

namespace {
// The two angles are arctangents, thus the module can only give a value in the open range -pi/2 to pi/2. An
// angle outside that range is not a value that the module can give, and the direction that it builds is not in
// the half-space. Thus it is not a regression input. This limit is a property of the two angles.
constexpr float kMaxAngle = 1.5707F;  // less than pi/2 by 5e-5 rad

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

// The direction range includes the origin and both half-spaces, thus the fuzz test also examines the directions
// that the module cannot use.
FUZZ_TEST(AxisToGimbalAnglesFuzz, regressionTestAxisToGimbalAngles)
    .WithDomains(xmera::fuzz::Vector3fInRange(-kMrpLimit, kMrpLimit),               // sigma_MB (MRP)
                 xmera::fuzz::Vector3fInRange(-kDirectionLimit, kDirectionLimit));  // direction, body frame

FUZZ_TEST(AxisToGimbalAnglesFuzz, regressionTestAxisToGimbalAnglesFromAngles)
    .WithDomains(xmera::fuzz::Vector3fInRange(-kMrpLimit, kMrpLimit),  // sigma_MB (MRP)
                 fuzztest::InRange(-kMaxAngle, kMaxAngle),             // angle1 [rad]
                 fuzztest::InRange(-kMaxAngle, kMaxAngle));            // angle2 [rad]

// ---------------------------------------------------------------------------
// Property fuzz tests
// ---------------------------------------------------------------------------

FUZZ_TEST(AxisToGimbalAnglesPropertyFuzz, propertyOutputIsUsable)
    .WithDomains(xmera::fuzz::Vector3fInRange(-kMrpLimit, kMrpLimit),
                 xmera::fuzz::Vector3fInRange(-kDirectionLimit, kDirectionLimit));

FUZZ_TEST(AxisToGimbalAnglesPropertyFuzz, propertyDirectionRecovered)
    .WithDomains(xmera::fuzz::Vector3fInRange(-kMrpLimit, kMrpLimit),
                 xmera::fuzz::Vector3fInRange(-kDirectionLimit, kDirectionLimit));

// The module makes the input direction a unit vector, thus the length has no effect. The range below keeps each
// scaled component in the normal float range, where the scaling keeps the direction.
FUZZ_TEST(AxisToGimbalAnglesPropertyFuzz, propertyLengthHasNoEffect)
    .WithDomains(xmera::fuzz::Vector3fInRange(-kMrpLimit, kMrpLimit),
                 xmera::fuzz::Vector3fInRange(-kDirectionLimit, kDirectionLimit),
                 fuzztest::InRange(1e-3F, 1e3F));
