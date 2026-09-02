#include "cssWeightedLeastSquaresAlgorithm_c.h"

#include "cssWeightedLeastSquaresAlgorithm.h"
#include "cssWeightedLeastSquaresTypes.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/opaqueHandle.h"

#include <Eigen/Core>

namespace {

/*! Build the validated C++ configuration from its C mirror. The boresight rows are copied element by
    element rather than mapped, because the C POD stores them as a two-dimensional array whose row
    layout must not be reinterpreted. */
CssWeightedLeastSquaresConfig configFromC(const CssWeightedLeastSquaresConstellation_c& constellation,
                                          const bool useWeights,
                                          const float sensorUseThresh) {
    Eigen::Matrix<float, kMaxNumCss, 3> cssNHat_B = Eigen::Matrix<float, kMaxNumCss, 3>::Zero();
    Eigen::Vector<float, kMaxNumCss> cssBias = Eigen::Vector<float, kMaxNumCss>::Zero();

    for (int sensor = 0; sensor < kMaxNumCss; ++sensor) {
        for (int component = 0; component < 3; ++component) {
            cssNHat_B(sensor, component) = constellation.cssNHat_B[sensor][component];
        }
        cssBias(sensor) = constellation.cssBias[sensor];
    }

    return CssWeightedLeastSquaresConfig::create(cssNHat_B, cssBias, constellation.numCss, useWeights, sensorUseThresh);
}

/*! Convert the algorithm's output struct to its C mirror. */
CssWeightedLeastSquaresOutput_c outputToC(const CssWeightedLeastSquaresOutput& out) {
    CssWeightedLeastSquaresOutput_c result{};
    eigenVectorToCArray(out.sunHeading_B, result.sunHeading_B.data);
    eigenVectorToCArray(out.omega_BN_B, result.omega_BN_B.data);
    eigenVectorToCArray(out.residualStateHeading, result.residualStateHeading.data);
    eigenVectorToCArray(out.postFitResiduals, result.postFitResiduals);
    result.numActiveCss = out.numActiveCss;
    return result;
}

}  // namespace

uint32_t CssWeightedLeastSquaresAlgorithm_getMaxNumCss(void) { return kMaxNumCss; }

bool CssWeightedLeastSquaresAlgorithm_validateConfig(const CssWeightedLeastSquaresConstellation_c* constellation,
                                                     const bool useWeights,
                                                     const float sensorUseThresh) {
    try {
        (void)configFromC(*constellation, useWeights, sensorUseThresh);
        return true;
    } catch (const fsw::invalid_argument&) {
        return false;
    }
}

CssWeightedLeastSquaresAlgorithmHandle* CssWeightedLeastSquaresAlgorithm_create(
    const CssWeightedLeastSquaresConstellation_c* constellation,
    const bool useWeights,
    const float sensorUseThresh) {
    return fsw::createHandle<::CssWeightedLeastSquaresAlgorithm, CssWeightedLeastSquaresAlgorithmHandle>(
        configFromC(*constellation, useWeights, sensorUseThresh));
}

void CssWeightedLeastSquaresAlgorithm_destroy(CssWeightedLeastSquaresAlgorithmHandle* self) {
    fsw::deleteHandle<::CssWeightedLeastSquaresAlgorithm>(self);
}

void CssWeightedLeastSquaresAlgorithm_setConfig(CssWeightedLeastSquaresAlgorithmHandle* self,
                                                const CssWeightedLeastSquaresConstellation_c* constellation,
                                                const bool useWeights,
                                                const float sensorUseThresh) {
    fsw::fromHandle<::CssWeightedLeastSquaresAlgorithm>(self)->setConfig(
        configFromC(*constellation, useWeights, sensorUseThresh));
}

void CssWeightedLeastSquaresAlgorithm_reInitialize(CssWeightedLeastSquaresAlgorithmHandle* self) {
    fsw::fromHandle<::CssWeightedLeastSquaresAlgorithm>(self)->reInitialize();
}

CssWeightedLeastSquaresOutput_c CssWeightedLeastSquaresAlgorithm_update(CssWeightedLeastSquaresAlgorithmHandle* self,
                                                                        const uint64_t callTime,
                                                                        const CssWeightedLeastSquaresInputs_c* inputs) {
    const CssWeightedLeastSquaresOutput out = fsw::fromHandle<::CssWeightedLeastSquaresAlgorithm>(self)->update(
        callTime, cArrayToEigenVector(inputs->cosValues));
    return outputToC(out);
}
