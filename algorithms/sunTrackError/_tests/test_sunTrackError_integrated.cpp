#include "sunTrackErrorIntegratedTestHelpers.hpp"

#include <Eigen/Core>
#include <numbers>

namespace {
constexpr uint64_t kHalfSecNs = 500000000ULL;                        // 0.5 s update period
constexpr float kManeuverRate = std::numbers::pi_v<float> / 180.0F;  // 1 deg/s feed-forward slew

// Fixed corrected-reference offset defining the R frame relative to the input reference R0.
const Eigen::Vector3f kSigmaR0R{0.01F, 0.05F, -0.55F};
// Representative Sun geometry (spacecraft and Sun inertial positions) that engages the
// avoidance maneuver.
const Eigen::Vector3f kRBN_N{-30.0F, 20.0F, -50.0F};
const Eigen::Vector3f kRSN_N{1.0F, 2.0F, 3.0F};
const Eigen::Vector3f kSensitiveHat_B{0.0F, -1.0F, 0.0F};
}  // namespace

// ---------------------------------------------------------------------------
// Plain attitude tracking error (no Sun-avoidance maneuver): the optional trans/ephemeris
// messages are absent, so computeAngleStart is false and the initial maneuver angle is zero.
// The corrected-reference offset (sigma_R0R) is still applied.
// ---------------------------------------------------------------------------
TEST(SunTrackErrorIntegrated, TrackingErrorOnly) {
    integratedRegression(kSigmaR0R,
                         Eigen::Vector3f::Zero(),  // sensitiveHat_B (unused)
                         0.0F,                     // angleRate
                         false,                    // computeAngleStart
                         Eigen::Vector3f::Zero(),  // r_BN_N (unused)
                         Eigen::Vector3f::Zero(),  // r_SN_N (unused)
                         kHalfSecNs,
                         12);
}

// ---------------------------------------------------------------------------
// Sun-avoidance maneuver over a short run: the residual maneuver angle is still being fed
// forward (relativeAngle > 0) throughout, exercising the catch-up rate term.
// ---------------------------------------------------------------------------
TEST(SunTrackErrorIntegrated, SunAvoidanceFeedingForward) {
    integratedRegression(kSigmaR0R, kSensitiveHat_B, kManeuverRate, true, kRBN_N, kRSN_N, kHalfSecNs, 12);
}

// ---------------------------------------------------------------------------
// Sun-avoidance maneuver over a long run: the residual angle decays to zero and stays
// clamped, exercising the relativeAngle > 0 -> 0 transition and the post-maneuver steady state.
// ---------------------------------------------------------------------------
TEST(SunTrackErrorIntegrated, SunAvoidanceDecaysToZero) {
    integratedRegression(kSigmaR0R, kSensitiveHat_B, kManeuverRate, true, kRBN_N, kRSN_N, kHalfSecNs, 400);
}
