#include "cssWeightedLeastSquaresAlgorithm_c.h"

#include "cssWeightedLeastSquaresAlgorithm.h"
#include "cssWeightedLeastSquaresTypes.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/opaqueHandle.h"

#include <Eigen/Core>
#include <array>

namespace {

/*! Build the validated C++ configuration from its C mirror. The boresight rows are copied element by
    element rather than mapped, because the C POD stores them as a two-dimensional array whose row
    layout must not be reinterpreted. */
CssWeightedLeastSquaresConfig configFromC(const CssWeightedLeastSquaresConstellation_c& constellation,
                                          const bool useMeasurementsAsWeights,
                                          const float sensorUseThresh,
                                          const float controlPeriod) {
    std::array<CssConfiguration, kMaxNumCssSensors> cssSensors{};

    for (size_t sensor = 0; sensor < static_cast<size_t>(kMaxNumCssSensors); ++sensor) {
        cssSensors.at(sensor).nHat_B = cArrayToEigenVector3<float>(constellation.cssSensors[sensor].nHat_B.data);
        cssSensors.at(sensor).availability = fsw::toDeviceAvailability(constellation.cssSensors[sensor].availability);
    }

    return CssWeightedLeastSquaresConfig::create(cssSensors, useMeasurementsAsWeights, sensorUseThresh, controlPeriod);
}

/*! Convert the algorithm's output struct to its C mirror. */
CssWeightedLeastSquaresOutput_c outputToC(const CssWeightedLeastSquaresOutput& out) {
    CssWeightedLeastSquaresOutput_c result{};
    eigenVectorToCArray(out.sunHeading_B, result.sunHeading_B.data);
    eigenVectorToCArray(out.omega_BN_B, result.omega_BN_B.data);
    eigenVectorToCArray(out.postFitResiduals, result.postFitResiduals);
    result.numCssViewingSun = out.numCssViewingSun;
    return result;
}

}  // namespace

uint32_t CssWeightedLeastSquaresAlgorithm_getMaxNumCss(void) { return kMaxNumCssSensors; }

bool CssWeightedLeastSquaresAlgorithm_validateConfig(const CssWeightedLeastSquaresConstellation_c* constellation,
                                                     const bool useMeasurementsAsWeights,
                                                     const float sensorUseThresh,
                                                     const float controlPeriod) {
    try {
        (void)configFromC(*constellation, useMeasurementsAsWeights, sensorUseThresh, controlPeriod);
        return true;
    } catch (const fsw::invalid_argument&) {
        return false;
    }
}

CssWeightedLeastSquaresAlgorithmHandle* CssWeightedLeastSquaresAlgorithm_create(
    const CssWeightedLeastSquaresConstellation_c* constellation,
    const bool useMeasurementsAsWeights,
    const float sensorUseThresh,
    const float controlPeriod) {
    return fsw::createHandle<::CssWeightedLeastSquaresAlgorithm, CssWeightedLeastSquaresAlgorithmHandle>(
        configFromC(*constellation, useMeasurementsAsWeights, sensorUseThresh, controlPeriod));
}

void CssWeightedLeastSquaresAlgorithm_destroy(CssWeightedLeastSquaresAlgorithmHandle* self) {
    fsw::deleteHandle<::CssWeightedLeastSquaresAlgorithm>(self);
}

void CssWeightedLeastSquaresAlgorithm_setConfig(CssWeightedLeastSquaresAlgorithmHandle* self,
                                                const CssWeightedLeastSquaresConstellation_c* constellation,
                                                const bool useMeasurementsAsWeights,
                                                const float sensorUseThresh,
                                                const float controlPeriod) {
    fsw::fromHandle<::CssWeightedLeastSquaresAlgorithm>(self)->setConfig(
        configFromC(*constellation, useMeasurementsAsWeights, sensorUseThresh, controlPeriod));
}

void CssWeightedLeastSquaresAlgorithm_reInitialize(CssWeightedLeastSquaresAlgorithmHandle* self) {
    fsw::fromHandle<::CssWeightedLeastSquaresAlgorithm>(self)->reInitialize();
}

CssWeightedLeastSquaresOutput_c CssWeightedLeastSquaresAlgorithm_update(CssWeightedLeastSquaresAlgorithmHandle* self,
                                                                        const CssWeightedLeastSquaresInputs_c* inputs) {
    const CssWeightedLeastSquaresOutput out =
        fsw::fromHandle<::CssWeightedLeastSquaresAlgorithm>(self)->update(cArrayToEigenVector(inputs->cosValues));
    return outputToC(out);
}
