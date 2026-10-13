#include "gimbalToMotorAnglesSplineAlgorithm_c.h"
#include "gimbalToMotorAnglesSplineAlgorithm.h"
#include "utilities/fsw/opaqueHandle.h"

namespace {
/*! Convert a C-shared rhombus POD into the algorithm's rhombus type. */
GimbalRhombusDomain toDomain(const GimbalRhombusDomain_c* src) {
    return GimbalRhombusDomain{.centerTipAngle = src->centerTipAngle,
                               .centerTiltAngle = src->centerTiltAngle,
                               .halfDiagonalTip = src->halfDiagonalTip,
                               .halfDiagonalTilt = src->halfDiagonalTilt};
}

/*! Convert a C-shared coefficient POD into the algorithm's std::array coefficient type. */
SplineCoefficients toCoefficients(const SplineCoefficients_c* src) {
    SplineCoefficients out{};
    for (std::size_t index = 0; index < out.size(); ++index) {
        out[index] = src->data[index];
    }
    return out;
}

/*! Build the validated C++ configuration from the flattened C parameters. */
GimbalToMotorAnglesSplineConfig makeConfig(const GimbalRhombusDomain_c* domain,
                                           const SplineCoefficients_c* coefficients) {
    return GimbalToMotorAnglesSplineConfig::create(toDomain(domain), toCoefficients(coefficients));
}
}  // namespace

uint32_t GimbalToMotorAnglesSplineAlgorithm_getNumSplineIntervals(void) { return NUM_SPLINE_INTERVALS; }

uint32_t GimbalToMotorAnglesSplineAlgorithm_getNumSplineCoeffs(void) { return NUM_SPLINE_COEFFS; }

bool GimbalToMotorAnglesSplineAlgorithm_validateConfig(const GimbalRhombusDomain_c* domain,
                                                       const SplineCoefficients_c* coefficients) {
    try {
        (void)makeConfig(domain, coefficients);
        return true;
    } catch (const fsw::invalid_argument&) {
        return false;
    }
}

GimbalToMotorAnglesSplineAlgorithmHandle* GimbalToMotorAnglesSplineAlgorithm_create(
    const GimbalRhombusDomain_c* domain,
    const SplineCoefficients_c* coefficients) {
    return fsw::createHandle<::GimbalToMotorAnglesSplineAlgorithm, GimbalToMotorAnglesSplineAlgorithmHandle>(
        makeConfig(domain, coefficients));
}

void GimbalToMotorAnglesSplineAlgorithm_destroy(GimbalToMotorAnglesSplineAlgorithmHandle* self) {
    fsw::deleteHandle<::GimbalToMotorAnglesSplineAlgorithm>(self);
}

void GimbalToMotorAnglesSplineAlgorithm_setConfig(GimbalToMotorAnglesSplineAlgorithmHandle* self,
                                                  const GimbalRhombusDomain_c* domain,
                                                  const SplineCoefficients_c* coefficients) {
    fsw::fromHandle<::GimbalToMotorAnglesSplineAlgorithm>(self)->setConfig(makeConfig(domain, coefficients));
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
