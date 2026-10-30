#include "gimbalToMotorAnglesSplineAlgorithm.h"

#include <math.h>

namespace {

/*! @brief Spline interval that holds a normalized coordinate, and the local coordinate in it. */
struct SplineIntervalLocation {
    uint32_t interval;  //!< [-] Interval index in [0, NUM_SPLINE_INTERVALS - 1]
    float t;            //!< [-] Local coordinate in the interval, in [0, 1]
};

/*! This method computes the four uniform cubic B-spline weights for the local coordinate in a spline interval.
 @return std::array<float, 4>
 @param t [-] Local coordinate in the interval, in [0, 1]
*/
std::array<float, 4> cubicBSplineWeights(const float t) {
    const float t2 = t * t;
    const float t3 = t2 * t;
    const float oneMinusT = 1.0F - t;
    return {oneMinusT * oneMinusT * oneMinusT / 6.0F,
            (3.0F * t3 - 6.0F * t2 + 4.0F) / 6.0F,
            (-3.0F * t3 + 3.0F * t2 + 3.0F * t + 1.0F) / 6.0F,
            t3 / 6.0F};
}

/*! This method finds the spline interval that holds a normalized coordinate and the local coordinate in it.
 @return SplineIntervalLocation
 @param x [-] Normalized coordinate in [-1, 1]
*/
SplineIntervalLocation splineInterval(const float x) {
    constexpr float h = 2.0F / static_cast<float>(kNumSplineIntervals);
    const float scaled = (x + 1.0F) / h;

    // Clamp interval at both ends. x = 1 must belong to the last interval. The clamp is done in float, before the
    // conversion, because converting a negative float to an unsigned integer is undefined behavior.
    const auto interval =
        static_cast<uint32_t>(std::clamp(floorf(scaled), 0.0F, static_cast<float>(kNumSplineIntervals - 1)));
    return {.interval = interval, .t = scaled - static_cast<float>(interval)};
}

/*! This method evaluates a uniform bicubic B-spline surface.
 @return float [steps] Surface value
 @param coefficients [steps] Control coefficients of the surface
 @param u [-] First normalized rhombus coordinate in [-1, 1]
 @param v [-] Second normalized rhombus coordinate in [-1, 1]
*/
float evaluateSurface(const SplineCoefficients& coefficients, const float u, const float v) {
    const auto [intervalU, tu] = splineInterval(u);
    const auto [intervalV, tv] = splineInterval(v);
    const std::array<float, 4> weightsU = cubicBSplineWeights(tu);
    const std::array<float, 4> weightsV = cubicBSplineWeights(tv);

    float value = 0.0F;
    for (uint32_t p = 0; p < 4; ++p) {
        const uint32_t rowStart = (intervalU + p) * kNumSplineCoeffsPerAxis + intervalV;
        float rowValue = 0.0F;
        for (uint32_t q = 0; q < 4; ++q) {
            rowValue += weightsV[q] * coefficients[rowStart + q];
        }
        value += weightsU[p] * rowValue;
    }
    return value;
}

}  // namespace

GimbalToMotorAnglesSplineAlgorithm::GimbalToMotorAnglesSplineAlgorithm(const GimbalToMotorAnglesSplineConfig& config)
    : cfg(config) {
    setConfig(config);
    reInitialize();
}

/*! Replaces the algorithm's configuration for runtime reconfiguration. The previous valid output is kept, so new
 default motor angles are used only at the next reInitialize().
 @param config Validated configuration
*/
void GimbalToMotorAnglesSplineAlgorithm::setConfig(const GimbalToMotorAnglesSplineConfig& config) {
    this->cfg = config;
}

/*! Resets the stored motor angles to the configured default motor angles.
 @return void
*/
void GimbalToMotorAnglesSplineAlgorithm::reInitialize() {
    this->previousValidOutput = {.motorAngle1 = this->cfg.getDefaultMotor1Angle(),
                                 .motorAngle2 = this->cfg.getDefaultMotor2Angle()};
}

/*! This method determines the stepper motor angles corresponding to the incoming gimbal tip and tilt angles.
 The spline surface of each motor gives motor steps, which are converted to motor angles with the step angle.
 Requests that are not finite or are outside the addressable rhombus return the previous valid motor angles.
 On the first call, the previous valid motor angles are the default motor angles.
 @return GimbalToMotorAnglesSplineOutput
 @param gimbalAngle1 [rad] Gimbal tip angle
 @param gimbalAngle2 [rad] Gimbal tilt angle
*/
GimbalToMotorAnglesSplineOutput GimbalToMotorAnglesSplineAlgorithm::update(float gimbalAngle1, float gimbalAngle2) {
    GimbalToMotorAnglesSplineOutput output{this->previousValidOutput};

    if (!fsw::is_finite(gimbalAngle1) || !fsw::is_finite(gimbalAngle2)) {
        return output;
    }

    // Map the gimbal angles to the normalized rhombus coordinates
    const GimbalRhombusDomain& domain = this->cfg.getDomain();
    const float x = (gimbalAngle1 - domain.centerTipAngle) / domain.halfDiagonalTip;
    const float y = (gimbalAngle2 - domain.centerTiltAngle) / domain.halfDiagonalTilt;
    float u = x + y;
    float v = x - y;

    // Reject requests outside the addressable rhombus. Finite inputs can still give a NaN u or v (an overflowed
    // x = +inf plus y = -inf), so the test is written to also reject NaN.
    constexpr float limit = 1.0F + kDomainTolerance;
    if (!(fabsf(u) <= limit && fabsf(v) <= limit)) {
        return output;
    }
    // Clamp the variables if they're just outside the rhombus but within kDomainTolerance
    u = std::clamp(u, -1.0F, 1.0F);
    v = std::clamp(v, -1.0F, 1.0F);

    // Each motor has its own surface. The surfaces give motor steps, which the step angle converts to motor angles.
    const float stepAngle = this->cfg.getStepAngle();
    output.motorAngle1 = evaluateSurface(this->cfg.getMotor1Coefficients(), u, v) * stepAngle;
    output.motorAngle2 = evaluateSurface(this->cfg.getMotor2Coefficients(), u, v) * stepAngle;

    // Store the valid motor angles for the next update call
    this->previousValidOutput = output;

    return output;
}
