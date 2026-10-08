#include "cobConverterAlgorithm_c.h"
#include "cobConverterAlgorithm.h"
#include "cobConverterTypes.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/opaqueHandle.h"

#include <Eigen/Core>

namespace {

//! Build the validated C++ configuration from the flattened C parameters.
CobConverterConfig makeConfig(float radius,
                              float radiusUncertainty,
                              Matrix3f_c attitudeCovariance,
                              float numStandardDeviations,
                              float standardDeviation,
                              bool specifiedStandardDeviation,
                              bool outlierDetectionEnabled,
                              CalibrationCoefficients_c calibrationCoefficients,
                              int32_t cameraId,
                              float fieldOfViewX,
                              float fieldOfViewY,
                              float resolutionX,
                              float resolutionY,
                              Vector3f_c bodyToCameraMrp) {
    const CalibrationCoefficients coefficients{.k1 = calibrationCoefficients.k1,
                                               .k2 = calibrationCoefficients.k2,
                                               .k3 = calibrationCoefficients.k3,
                                               .p1 = calibrationCoefficients.p1,
                                               .p2 = calibrationCoefficients.p2};
    return CobConverterConfig::create(radius,
                                      radiusUncertainty,
                                      c2DArrayToEigenMatrix3(attitudeCovariance.data),
                                      numStandardDeviations,
                                      standardDeviation,
                                      specifiedStandardDeviation,
                                      outlierDetectionEnabled,
                                      coefficients,
                                      cameraId,
                                      fieldOfViewX,
                                      fieldOfViewY,
                                      resolutionX,
                                      resolutionY,
                                      cArrayToEigenVector3<float>(bodyToCameraMrp.data));
}

CobConverterOutput_c outputToC(const CobConverterOutput& out, const CobConverterDiagnosticOutput& diag) {
    CobConverterOutput_c result{};
    eigenMatrixToCArray2D(out.covar_N, result.covar_N.data);
    eigenVectorToCArray(out.rhat_BN_N, result.rhat_BN_N.data);
    result.unitVecTimeTag = out.unitVecTimeTag;
    result.unitVecValid = out.unitVecValid;
    eigenMatrixToCArray2D(diag.covar_C, result.covar_C.data);
    eigenMatrixToCArray2D(diag.covar_B, result.covar_B.data);
    eigenVectorToCArray(diag.rhat_BN_C, result.rhat_BN_C.data);
    eigenVectorToCArray(diag.rhat_BN_B, result.rhat_BN_B.data);
    eigenVectorToCArray(diag.rhat_COB_C, result.rhat_COB_C.data);
    eigenVectorToCArray(diag.rhat_COB_N, result.rhat_COB_N.data);
    eigenVectorToCArray(diag.rhat_COB_B, result.rhat_COB_B.data);
    eigenVectorToCArray(diag.centerOfBrightness, result.centerOfBrightness.data);
    eigenVectorToCArray(diag.centerOfMass, result.centerOfMass.data);
    result.offsetFactor = diag.offsetFactor;
    result.objectPixelRadius = diag.objectPixelRadius;
    result.phaseAngle = diag.phaseAngle;
    result.sunDirection = diag.sunDirection;
    result.comTimeTag = diag.comTimeTag;
    result.comValid = diag.comValid;
    result.comErrorOutlierTrigger = diag.comErrorOutlierTrigger;
    result.brownConradyCOMValid = diag.brownConradyCOMValid;
    result.brownConradyCOBValid = diag.brownConradyCOBValid;
    return result;
}

}  // namespace

bool CobConverterAlgorithm_validateConfig(float radius,
                                          float radiusUncertainty,
                                          Matrix3f_c attitudeCovariance,
                                          float numStandardDeviations,
                                          float standardDeviation,
                                          bool specifiedStandardDeviation,
                                          bool outlierDetectionEnabled,
                                          CalibrationCoefficients_c calibrationCoefficients,
                                          int32_t cameraId,
                                          float fieldOfViewX,
                                          float fieldOfViewY,
                                          float resolutionX,
                                          float resolutionY,
                                          Vector3f_c bodyToCameraMrp) {
    try {
        (void)makeConfig(radius,
                         radiusUncertainty,
                         attitudeCovariance,
                         numStandardDeviations,
                         standardDeviation,
                         specifiedStandardDeviation,
                         outlierDetectionEnabled,
                         calibrationCoefficients,
                         cameraId,
                         fieldOfViewX,
                         fieldOfViewY,
                         resolutionX,
                         resolutionY,
                         bodyToCameraMrp);
        return true;
    } catch (const fsw::invalid_argument&) {
        return false;
    }
}

CobConverterAlgorithmHandle* CobConverterAlgorithm_create(float radius,
                                                          float radiusUncertainty,
                                                          Matrix3f_c attitudeCovariance,
                                                          float numStandardDeviations,
                                                          float standardDeviation,
                                                          bool specifiedStandardDeviation,
                                                          bool outlierDetectionEnabled,
                                                          CalibrationCoefficients_c calibrationCoefficients,
                                                          int32_t cameraId,
                                                          float fieldOfViewX,
                                                          float fieldOfViewY,
                                                          float resolutionX,
                                                          float resolutionY,
                                                          Vector3f_c bodyToCameraMrp) {
    return fsw::createHandle<::CobConverterAlgorithm, CobConverterAlgorithmHandle>(
        makeConfig(radius,
                   radiusUncertainty,
                   attitudeCovariance,
                   numStandardDeviations,
                   standardDeviation,
                   specifiedStandardDeviation,
                   outlierDetectionEnabled,
                   calibrationCoefficients,
                   cameraId,
                   fieldOfViewX,
                   fieldOfViewY,
                   resolutionX,
                   resolutionY,
                   bodyToCameraMrp));
}

void CobConverterAlgorithm_destroy(CobConverterAlgorithmHandle* self) {
    fsw::deleteHandle<::CobConverterAlgorithm>(self);
}

void CobConverterAlgorithm_setConfig(CobConverterAlgorithmHandle* self,
                                     float radius,
                                     float radiusUncertainty,
                                     Matrix3f_c attitudeCovariance,
                                     float numStandardDeviations,
                                     float standardDeviation,
                                     bool specifiedStandardDeviation,
                                     bool outlierDetectionEnabled,
                                     CalibrationCoefficients_c calibrationCoefficients,
                                     int32_t cameraId,
                                     float fieldOfViewX,
                                     float fieldOfViewY,
                                     float resolutionX,
                                     float resolutionY,
                                     Vector3f_c bodyToCameraMrp) {
    fsw::fromHandle<::CobConverterAlgorithm>(self)->setConfig(makeConfig(radius,
                                                                         radiusUncertainty,
                                                                         attitudeCovariance,
                                                                         numStandardDeviations,
                                                                         standardDeviation,
                                                                         specifiedStandardDeviation,
                                                                         outlierDetectionEnabled,
                                                                         calibrationCoefficients,
                                                                         cameraId,
                                                                         fieldOfViewX,
                                                                         fieldOfViewY,
                                                                         resolutionX,
                                                                         resolutionY,
                                                                         bodyToCameraMrp));
}

CobConverterOutput_c CobConverterAlgorithm_updateState(CobConverterAlgorithmHandle* self,
                                                       bool cobValid,
                                                       int32_t cobPixelsFound,
                                                       Vector2f_c cobCenterOfBrightness,
                                                       uint64_t cobTimeTag,
                                                       Vector3f_c sigma_BN,
                                                       Vector3f_c vehSunPntBdy,
                                                       Vector3d_c filterVehPosition,
                                                       Matrix3d_c filterVehPositionCovariance) {
    const CobMeasurement cob{
        .cobValid = cobValid,
        .cobPixelsFound = cobPixelsFound,
        .cobCenterOfBrightness = cArrayToEigenVector(cobCenterOfBrightness.data),
        .cobTimeTag = cobTimeTag,
    };
    const VehicleAttitude attitude{
        .sigma_BN = cArrayToEigenVector3<float>(sigma_BN.data),
        .vehSunPntBdy = cArrayToEigenVector3<float>(vehSunPntBdy.data),
    };
    const FilterState filter{
        .filterVehPosition = cArrayToEigenVector3<double>(filterVehPosition.data),
        .filterVehPositionCovariance = c2DArrayToEigenMatrix3<double>(filterVehPositionCovariance.data),
    };

    const auto [out, diag] = fsw::fromHandle<::CobConverterAlgorithm>(self)->updateState(cob, attitude, filter);
    return outputToC(out, diag);
}
