#include "sunAvoidanceIntegratedTestHelpers.hpp"

#include <Eigen/Core>
#include <numbers>

namespace {
constexpr uint64_t kHalfSecNs = 500000000ULL;                        // 0.5 s update period
constexpr float kManeuverRate = std::numbers::pi_v<float> / 180.0F;  // 1 deg/s feed-forward slew

// Representative Sun geometry (spacecraft and Sun inertial positions) that engages the
// avoidance maneuver.
const Eigen::Vector3d kRBN_N{-30.0, 20.0, -50.0};
const Eigen::Vector3d kRSN_N{1.0, 2.0, 3.0};
const Eigen::Vector3f kSensitiveHat_B{0.0F, -1.0F, 0.0F};
}  // namespace

// ---------------------------------------------------------------------------
// Plain attitude tracking error (no Sun-avoidance maneuver): a zero Sun position leaves no usable Sun
// direction, so the initial maneuver angle is zero and the adjusted reference passes through.
// ---------------------------------------------------------------------------
TEST(SunAvoidanceIntegrated, TrackingErrorOnly) {
    integratedRegression(kSensitiveHat_B,
                         kManeuverRate,            // unused: the maneuver angle starts at zero
                         Eigen::Vector3d::Zero(),  // r_BN_N
                         Eigen::Vector3d::Zero(),  // r_SN_N: no Sun information
                         kHalfSecNs,
                         12);
}

// ---------------------------------------------------------------------------
// Sun-avoidance maneuver over a short run: the residual maneuver angle is still being fed
// forward (relativeAngle > 0) throughout, exercising the catch-up rate term.
// ---------------------------------------------------------------------------
TEST(SunAvoidanceIntegrated, SunAvoidanceFeedingForward) {
    integratedRegression(kSensitiveHat_B, kManeuverRate, kRBN_N, kRSN_N, kHalfSecNs, 12);
}

// ---------------------------------------------------------------------------
// Sun-avoidance maneuver over a long run: the residual angle decays to zero and stays
// clamped, exercising the relativeAngle > 0 -> 0 transition and the post-maneuver steady state.
// ---------------------------------------------------------------------------
TEST(SunAvoidanceIntegrated, SunAvoidanceDecaysToZero) {
    integratedRegression(kSensitiveHat_B, kManeuverRate, kRBN_N, kRSN_N, kHalfSecNs, 400);
}
