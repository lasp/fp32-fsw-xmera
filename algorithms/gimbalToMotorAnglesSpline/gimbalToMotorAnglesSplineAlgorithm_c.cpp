#include "gimbalToMotorAnglesSplineAlgorithm_c.h"
#include "gimbalToMotorAnglesSplineAlgorithm.h"
#include "utilities/fsw/opaqueHandle.h"

namespace {
/*! Convert a C-shared rhombus POD into the algorithm's rhombus type. */
GimbalRhombusDomain toDomain(const GimbalRhombusDomain_c* src) {
    return GimbalRhombusDomain{
        .halfDiagonalTip = src->halfDiagonalTip, .halfDiagonalTilt = src->halfDiagonalTilt, .thetaMax = src->thetaMax};
}

/*! Convert a C-shared coefficient POD into the algorithm's std::array coefficient type. */
SplineCoefficients toCoefficients(const SplineCoefficients_c* src) {
    SplineCoefficients out{};
    for (std::size_t index = 0; index < out.size(); ++index) {
        out.at(index) = src->data[index];
    }
    return out;
}

/*! Build the validated C++ configuration from the flattened C parameters. */
GimbalToMotorAnglesSplineConfig makeConfig(const uint32_t stepsPerRevolution,
                                           const GimbalRhombusDomain_c* domain,
                                           const SplineCoefficients_c* motor1Coefficients,
                                           const SplineCoefficients_c* motor2Coefficients) {
    return GimbalToMotorAnglesSplineConfig::create(
        stepsPerRevolution, toDomain(domain), toCoefficients(motor1Coefficients), toCoefficients(motor2Coefficients));
}
}  // namespace

uint32_t GimbalToMotorAnglesSplineAlgorithm_getNumSplineIntervals(void) { return NUM_SPLINE_INTERVALS; }

uint32_t GimbalToMotorAnglesSplineAlgorithm_getNumSplineCoeffs(void) { return NUM_SPLINE_COEFFS; }

bool GimbalToMotorAnglesSplineAlgorithm_validateConfig(const uint32_t stepsPerRevolution,
                                                       const GimbalRhombusDomain_c* domain,
                                                       const SplineCoefficients_c* motor1Coefficients,
                                                       const SplineCoefficients_c* motor2Coefficients) {
    try {
        (void)makeConfig(stepsPerRevolution, domain, motor1Coefficients, motor2Coefficients);
        return true;
    } catch (const fsw::invalid_argument&) {
        return false;
    }
}

GimbalToMotorAnglesSplineAlgorithmHandle* GimbalToMotorAnglesSplineAlgorithm_create(
    const uint32_t stepsPerRevolution,
    const GimbalRhombusDomain_c* domain,
    const SplineCoefficients_c* motor1Coefficients,
    const SplineCoefficients_c* motor2Coefficients) {
    return fsw::createHandle<::GimbalToMotorAnglesSplineAlgorithm, GimbalToMotorAnglesSplineAlgorithmHandle>(
        makeConfig(stepsPerRevolution, domain, motor1Coefficients, motor2Coefficients));
}

void GimbalToMotorAnglesSplineAlgorithm_destroy(GimbalToMotorAnglesSplineAlgorithmHandle* self) {
    fsw::deleteHandle<::GimbalToMotorAnglesSplineAlgorithm>(self);
}

void GimbalToMotorAnglesSplineAlgorithm_setConfig(GimbalToMotorAnglesSplineAlgorithmHandle* self,
                                                  const uint32_t stepsPerRevolution,
                                                  const GimbalRhombusDomain_c* domain,
                                                  const SplineCoefficients_c* motor1Coefficients,
                                                  const SplineCoefficients_c* motor2Coefficients) {
    fsw::fromHandle<::GimbalToMotorAnglesSplineAlgorithm>(self)->setConfig(
        makeConfig(stepsPerRevolution, domain, motor1Coefficients, motor2Coefficients));
}

void GimbalToMotorAnglesSplineAlgorithm_reInitialize(GimbalToMotorAnglesSplineAlgorithmHandle* self) {
    fsw::fromHandle<::GimbalToMotorAnglesSplineAlgorithm>(self)->reInitialize();
}

GimbalToMotorAnglesSplineOutput_c GimbalToMotorAnglesSplineAlgorithm_update(
    GimbalToMotorAnglesSplineAlgorithmHandle* self,
    const float gimbalAngle1,
    const float gimbalAngle2) {
    const GimbalToMotorAnglesSplineOutput out =
        fsw::fromHandle<::GimbalToMotorAnglesSplineAlgorithm>(self)->update(gimbalAngle1, gimbalAngle2);

    GimbalToMotorAnglesSplineOutput_c result{};
    result.motorAngle1 = out.motorAngle1;
    result.motorAngle2 = out.motorAngle2;
    return result;
}
