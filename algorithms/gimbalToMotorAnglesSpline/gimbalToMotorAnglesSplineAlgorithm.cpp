#include "gimbalToMotorAnglesSplineAlgorithm.h"

#include <math.h>

namespace {

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
 @return int [-] Interval index in [0, NUM_SPLINE_INTERVALS - 1]
 @param x [-] Normalized coordinate in [-1, 1]
 @param t [-] Output local coordinate in the interval
*/
int splineInterval(const float x, float& t) {
    const float scaled = (x + 1.0F) * (static_cast<float>(kNumSplineIntervals) / 2.0F);
    int interval = static_cast<int>(floorf(scaled));
    // x = 1 belongs to the last interval
    if (interval < 0) {
        interval = 0;
    } else if (interval > kNumSplineIntervals - 1) {
        interval = kNumSplineIntervals - 1;
    }
    t = scaled - static_cast<float>(interval);
    return interval;
}

}  // namespace

GimbalToMotorAnglesSplineAlgorithm::GimbalToMotorAnglesSplineAlgorithm(const GimbalToMotorAnglesSplineConfig& config)
    : cfg(config) {
    setConfig(config);
    reInitialize();
}

/*! Replaces the algorithm's configuration for runtime reconfiguration. The previous valid output is kept.
 @param config Validated configuration
*/
void GimbalToMotorAnglesSplineAlgorithm::setConfig(const GimbalToMotorAnglesSplineConfig& config) {
    this->cfg = config;
}

/*! Resets the stored motor angles to the default motor angles.
 @return void
*/
void GimbalToMotorAnglesSplineAlgorithm::reInitialize() {
    this->previousValidOutput = GimbalToMotorAnglesSplineOutput{};
}

/*! This method determines the stepper motor angles corresponding to the incoming gimbal tip and tilt angles.
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
    if (!(fabsf(u) <= limit && fabsf(v) <= limit)) {  // NOLINT(readability-simplify-boolean-expr)
        return output;
    }
    u = fminf(fmaxf(u, -1.0F), 1.0F);
    v = fminf(fmaxf(v, -1.0F), 1.0F);

    // Motor 2 is the mirror of motor 1 about the tilt center, which swaps u and v
    output.motorAngle1 = this->evaluateSurface(u, v);
    output.motorAngle2 = this->evaluateSurface(v, u);

    // Store the valid motor angles for the next update call
    this->previousValidOutput = output;

    return output;
}

/*! This method evaluates the shared uniform bicubic B-spline surface.
 @return float [rad] Surface value
 @param u [-] First normalized rhombus coordinate in [-1, 1]
 @param v [-] Second normalized rhombus coordinate in [-1, 1]
*/
float GimbalToMotorAnglesSplineAlgorithm::evaluateSurface(const float u, const float v) const {
    float tu{};
    float tv{};
    const int intervalU = splineInterval(u, tu);
    const int intervalV = splineInterval(v, tv);
    const std::array<float, 4> weightsU = cubicBSplineWeights(tu);
    const std::array<float, 4> weightsV = cubicBSplineWeights(tv);

    const SplineCoefficients& coefficients = this->cfg.getCoefficients();
    float value = 0.0F;
    for (int p = 0; p < 4; ++p) {
        const int rowStart = ((intervalU + p) * kNumSplineCoeffsPerAxis) + intervalV;
        float rowValue = 0.0F;
        for (int q = 0; q < 4; ++q) {
            rowValue += weightsV[q] * coefficients[rowStart + q];
        }
        value += weightsU[p] * rowValue;
    }
    return value;
}
