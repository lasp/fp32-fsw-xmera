#include "cobConverterAlgorithm_c.h"
#include "cobConverterAlgorithm.h"
#include "cobConverterTypes.h"
#include "utilities/fsw/eigenSupport.h"

#include <Eigen/Core>

namespace {

CobConverterConfig configFromC(const CobConverterConfig_c& c) {
    const CalibrationCoefficients calibrationCoefficients{c.calibrationCoefficients.k1,
                                                          c.calibrationCoefficients.k2,
                                                          c.calibrationCoefficients.k3,
                                                          c.calibrationCoefficients.p1,
                                                          c.calibrationCoefficients.p2};
    return CobConverterConfig::create(static_cast<PhaseAngleCorrectionMethodAlgorithm>(c.phaseAngleCorrectionMethod),
                                      c.radius,
                                      c.radiusUncertainty,
                                      c2DArrayToEigenMatrix3(c.attitudeCovariance.data),
                                      c.numStandardDeviations,
                                      c.standardDeviation,
                                      c.specifiedStandardDeviation,
                                      c.outlierDetectionEnabled,
                                      calibrationCoefficients,
                                      c.cameraId,
                                      c.fieldOfView,
                                      c.resolutionX,
                                      c.resolutionY,
                                      cArrayToEigenVector3<float>(c.bodyToCameraMrp.data));
}

CobConverterInput inputFromC(const CobConverterInput_c& c) {
    CobConverterInput input;
    input.cobValid = c.cobValid;
    input.cobPixelsFound = c.cobPixelsFound;
    input.cobCenterOfBrightness = cArrayToEigenVector(c.cobCenterOfBrightness.data);
    input.cobTimeTag = c.cobTimeTag;
    input.sigma_BN = cArrayToEigenVector3<float>(c.sigma_BN.data);
    input.vehSunPntBdy = cArrayToEigenVector3<float>(c.vehSunPntBdy.data);
    input.filterVehPosition = cArrayToEigenVector3<double>(c.filterVehPosition.data);
    input.filterVehPositionCovariance = c2DArrayToEigenMatrix3<double>(c.filterVehPositionCovariance.data);
    return input;
}

CobConverterOutput_c outputToC(const CobConverterOutput& out) {
    CobConverterOutput_c result{};
    eigenMatrixToCArray2D(out.covar_N, result.covar_N.data);
    eigenMatrixToCArray2D(out.covar_C, result.covar_C.data);
    eigenMatrixToCArray2D(out.covar_B, result.covar_B.data);
    eigenVectorToCArray(out.rhat_BN_N, result.rhat_BN_N.data);
    eigenVectorToCArray(out.rhat_BN_C, result.rhat_BN_C.data);
    eigenVectorToCArray(out.rhat_BN_B, result.rhat_BN_B.data);
    result.unitVecTimeTag = out.unitVecTimeTag;
    result.unitVecValid = out.unitVecValid;
    eigenVectorToCArray(out.centerOfBrightness, result.centerOfBrightness.data);
    eigenVectorToCArray(out.centerOfMass, result.centerOfMass.data);
    result.offsetFactor = out.offsetFactor;
    result.objectPixelRadius = out.objectPixelRadius;
    result.phaseAngle = out.phaseAngle;
    result.sunDirection = out.sunDirection;
    result.comTimeTag = out.comTimeTag;
    result.comValid = out.comValid;
    result.coberrorOutlierTrigger = out.coberrorOutlierTrigger;
    return result;
}

}  // namespace

CobConverterAlgorithmHandle* CobConverterAlgorithm_create(const CobConverterConfig_c* config) {
    return reinterpret_cast<CobConverterAlgorithmHandle*>(new ::CobConverterAlgorithm(configFromC(*config)));
}

void CobConverterAlgorithm_destroy(CobConverterAlgorithmHandle* self) {
    delete reinterpret_cast<::CobConverterAlgorithm*>(self);
}

void CobConverterAlgorithm_setConfig(CobConverterAlgorithmHandle* self, const CobConverterConfig_c* config) {
    reinterpret_cast<::CobConverterAlgorithm*>(self)->setConfig(configFromC(*config));
}

CobConverterOutput_c CobConverterAlgorithm_updateState(CobConverterAlgorithmHandle* self,
                                                       const CobConverterInput_c* input) {
    const CobConverterOutput out = reinterpret_cast<::CobConverterAlgorithm*>(self)->updateState(inputFromC(*input));
    return outputToC(out);
}
