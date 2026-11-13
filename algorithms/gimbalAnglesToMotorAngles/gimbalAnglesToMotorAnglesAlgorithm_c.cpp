#include "gimbalAnglesToMotorAnglesAlgorithm_c.h"
#include "gimbalAnglesToMotorAnglesAlgorithm.h"
#include "utilities/fsw/opaqueHandle.h"

namespace {
/*! Convert a C-shared coefficient POD into the algorithm's std::array coefficient type. */
SplineCoefficients toCoefficients(const SplineCoefficients_c* src) {
    SplineCoefficients out{};
    for (std::size_t index = 0; index < out.size(); ++index) {
        out.at(index) = src->data[index];
    }
    return out;
}

/*! Build the validated C++ configuration from the flattened C parameters. */
GimbalAnglesToMotorAnglesConfig makeConfig(const uint32_t stepsPerRevolution,
                                           const float halfDiagonalTip,
                                           const float halfDiagonalTilt,
                                           const float thetaMax,
                                           const SplineCoefficients_c* motor1Coefficients,
                                           const SplineCoefficients_c* motor2Coefficients) {
    return GimbalAnglesToMotorAnglesConfig::create(
        stepsPerRevolution,
        GimbalRhombusDomain{
            .halfDiagonalTip = halfDiagonalTip, .halfDiagonalTilt = halfDiagonalTilt, .thetaMax = thetaMax},
        toCoefficients(motor1Coefficients),
        toCoefficients(motor2Coefficients));
}
}  // namespace

uint32_t GimbalAnglesToMotorAnglesAlgorithm_getNumSplineIntervals(void) { return NUM_SPLINE_INTERVALS; }

uint32_t GimbalAnglesToMotorAnglesAlgorithm_getNumSplineCoeffs(void) { return NUM_SPLINE_COEFFS; }

bool GimbalAnglesToMotorAnglesAlgorithm_validateConfig(const uint32_t stepsPerRevolution,
                                                       const float halfDiagonalTip,
                                                       const float halfDiagonalTilt,
                                                       const float thetaMax,
                                                       const SplineCoefficients_c* motor1Coefficients,
                                                       const SplineCoefficients_c* motor2Coefficients) {
    try {
        (void)makeConfig(
            stepsPerRevolution, halfDiagonalTip, halfDiagonalTilt, thetaMax, motor1Coefficients, motor2Coefficients);
        return true;
    } catch (const fsw::invalid_argument&) {
        return false;
    }
}

GimbalAnglesToMotorAnglesAlgorithmHandle* GimbalAnglesToMotorAnglesAlgorithm_create(
    const uint32_t stepsPerRevolution,
    const float halfDiagonalTip,
    const float halfDiagonalTilt,
    const float thetaMax,
    const SplineCoefficients_c* motor1Coefficients,
    const SplineCoefficients_c* motor2Coefficients) {
    return fsw::createHandle<::GimbalAnglesToMotorAnglesAlgorithm, GimbalAnglesToMotorAnglesAlgorithmHandle>(makeConfig(
        stepsPerRevolution, halfDiagonalTip, halfDiagonalTilt, thetaMax, motor1Coefficients, motor2Coefficients));
}

void GimbalAnglesToMotorAnglesAlgorithm_destroy(GimbalAnglesToMotorAnglesAlgorithmHandle* self) {
    fsw::deleteHandle<::GimbalAnglesToMotorAnglesAlgorithm>(self);
}

void GimbalAnglesToMotorAnglesAlgorithm_setConfig(GimbalAnglesToMotorAnglesAlgorithmHandle* self,
                                                  const uint32_t stepsPerRevolution,
                                                  const float halfDiagonalTip,
                                                  const float halfDiagonalTilt,
                                                  const float thetaMax,
                                                  const SplineCoefficients_c* motor1Coefficients,
                                                  const SplineCoefficients_c* motor2Coefficients) {
    fsw::fromHandle<::GimbalAnglesToMotorAnglesAlgorithm>(self)->setConfig(makeConfig(
        stepsPerRevolution, halfDiagonalTip, halfDiagonalTilt, thetaMax, motor1Coefficients, motor2Coefficients));
}

void GimbalAnglesToMotorAnglesAlgorithm_reInitialize(GimbalAnglesToMotorAnglesAlgorithmHandle* self) {
    fsw::fromHandle<::GimbalAnglesToMotorAnglesAlgorithm>(self)->reInitialize();
}

GimbalAnglesToMotorAnglesOutput_c GimbalAnglesToMotorAnglesAlgorithm_update(
    GimbalAnglesToMotorAnglesAlgorithmHandle* self,
    const float gimbalAngle1,
    const float gimbalAngle2) {
    const GimbalAnglesToMotorAnglesOutput out =
        fsw::fromHandle<::GimbalAnglesToMotorAnglesAlgorithm>(self)->update(gimbalAngle1, gimbalAngle2);

    GimbalAnglesToMotorAnglesOutput_c result{};
    result.motorAngle1 = out.motorAngle1;
    result.motorAngle2 = out.motorAngle2;
    return result;
}
