#include "gimbalAnglesToMotorAnglesAlgorithm.h"

#include <math.h>

namespace gimbal_angles_to_motor_angles_detail {

std::array<float, 4> cubicBSplineWeights(const float t) {
    const float t2 = t * t;
    const float t3 = t2 * t;
    const float oneMinusT = 1.0F - t;
    return {oneMinusT * oneMinusT * oneMinusT / 6.0F,
            (3.0F * t3 - 6.0F * t2 + 4.0F) / 6.0F,
            (-3.0F * t3 + 3.0F * t2 + 3.0F * t + 1.0F) / 6.0F,
            t3 / 6.0F};
}

SplineIntervalLocation splineInterval(const float x) {
    constexpr float h = 2.0F / static_cast<float>(kNumSplineIntervals);
    const float scaled = (x + 1.0F) / h;

    // Clamp interval at both ends. x = 1 must belong to the last interval. The clamp is done in float, before the
    // conversion, because converting a negative float to an unsigned integer is undefined behavior.
    const auto interval =
        static_cast<uint32_t>(std::clamp(floorf(scaled), 0.0F, static_cast<float>(kNumSplineIntervals - 1)));
    return {.interval = interval, .t = scaled - static_cast<float>(interval)};
}

float evaluateSurface(const SplineCoefficients& coefficients, const float u, const float v) {
    const auto [intervalU, tu] = splineInterval(u);
    const auto [intervalV, tv] = splineInterval(v);
    const std::array<float, 4> weightsU = cubicBSplineWeights(tu);
    const std::array<float, 4> weightsV = cubicBSplineWeights(tv);

    float value = 0.0F;
    for (uint32_t p = 0; p < 4; ++p) {
        const uint32_t rowStart = ((intervalU + p) * kNumSplineCoeffsPerAxis) + intervalV;
        float rowValue = 0.0F;
        for (uint32_t q = 0; q < 4; ++q) {
            rowValue += weightsV.at(q) * coefficients.at(rowStart + q);
        }
        value += weightsU.at(p) * rowValue;
    }
    return value;
}

}  // namespace gimbal_angles_to_motor_angles_detail

/*! This method evaluates the motor 1 and motor 2 step surfaces and converts the motor steps to motor angles.
 @return GimbalAnglesToMotorAnglesOutput
 @param u [-] First normalized rhombus coordinate in [-1, 1]
 @param v [-] Second normalized rhombus coordinate in [-1, 1]
*/
GimbalAnglesToMotorAnglesOutput GimbalAnglesToMotorAnglesAlgorithm::motorAngles(float u, float v) const {
    return {
        .motorAngle1 = gimbal_angles_to_motor_angles_detail::evaluateSurface(this->cfg.getMotor1Coefficients(), u, v) *
                       this->stepAngle,
        .motorAngle2 = gimbal_angles_to_motor_angles_detail::evaluateSurface(this->cfg.getMotor2Coefficients(), u, v) *
                       this->stepAngle,
    };
}

GimbalAnglesToMotorAnglesAlgorithm::GimbalAnglesToMotorAnglesAlgorithm(const GimbalAnglesToMotorAnglesConfig& config)
    : cfg(config) {
    setConfig(config);
    reInitialize();
}

/*! Replaces the algorithm's configuration for runtime reconfiguration. The previous valid output is kept. If there is
 no previous valid output, the motor angles of the new configuration at zero gimbal angles are used at once.
 @param config Validated configuration
*/
void GimbalAnglesToMotorAnglesAlgorithm::setConfig(const GimbalAnglesToMotorAnglesConfig& config) {
    this->cfg = config;
    this->stepAngle = 2.0F * std::numbers::pi_v<float> / static_cast<float>(this->cfg.getStepsPerRevolution());
    this->zeroGimbalMotorAngles = this->motorAngles(0.0F, 0.0F);
}

/*! Clears the previous valid output, so the motor angles of the current configuration at zero gimbal angles are used
 until the next valid request.
 @return void
*/
void GimbalAnglesToMotorAnglesAlgorithm::reInitialize() { this->previousValidOutput.reset(); }

/*! This method determines the stepper motor angles corresponding to the incoming gimbal tip and tilt angles.
 The spline surface of each motor gives motor steps, which are converted to motor angles with the step angle
 2*pi / stepsPerRevolution.
 Requests that are not finite or are outside the addressable rhombus return the previous valid motor angles. If
 there is no previous valid output, they return the motor angles of the current configuration at zero gimbal angles.
 @return GimbalAnglesToMotorAnglesOutput
 @param gimbalAngle1 [rad] Gimbal tip angle
 @param gimbalAngle2 [rad] Gimbal tilt angle
*/
GimbalAnglesToMotorAnglesOutput GimbalAnglesToMotorAnglesAlgorithm::update(float gimbalAngle1, float gimbalAngle2) {
    // Map the gimbal angles to the normalized rhombus coordinates
    const GimbalRhombusDomain& domain = this->cfg.getDomain();
    const float x = gimbalAngle1 / domain.halfDiagonalTip;
    const float y = gimbalAngle2 / domain.halfDiagonalTilt;
    const float u = x + y;
    const float v = x - y;

    // Reject requests outside the addressable rhombus
    GimbalAnglesToMotorAnglesOutput output{};
    if (fabsf(u) <= 1.0F && fabsf(v) <= 1.0F) {
        // Store the valid motor angles for the next update call
        output = this->motorAngles(u, v);
        this->previousValidOutput = output;
    } else if (this->previousValidOutput.has_value()) {
        output = *this->previousValidOutput;
    } else {
        // Before the first valid request, use the motor angles at zero gimbal angles. They are not stored as the
        // previous valid output, so a later setConfig() takes effect at once.
        output = this->zeroGimbalMotorAngles;
    }

    return output;
}
