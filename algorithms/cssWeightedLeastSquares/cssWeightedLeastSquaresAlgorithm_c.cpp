#include "cssWeightedLeastSquaresAlgorithm_c.h"

#include "cssWeightedLeastSquaresAlgorithm.h"
#include "cssWeightedLeastSquaresTypes.h"
#include "utilities/fsw/deviceAvailability.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/opaqueHandle.h"

#include <Eigen/Core>
#include <array>
#include <cstdint>

namespace {

// Reassemble the flattened C arguments into the C++ configuration structs. The flat argument
// list is the shape of the extern "C" boundary only; everything behind this helper is struct
// based, and CssWeightedLeastSquaresConfig::create remains the single validation authority.
CssWeightedLeastSquaresConfig configFromC(const CssBoresightArray_c& cssNHat_B,
                                          const CssAvailabilityArray_c& cssAvailability,
                                          const bool useMeasurementsAsWeights,
                                          const float sensorUseThresh,
                                          const float controlPeriod) {
    std::array<CssConfiguration, kMaxNumCssSensors> cssSensors{};

    for (uint32_t sensor = 0; sensor < kMaxNumCssSensors; ++sensor) {
        for (uint32_t axis = 0; axis < 3U; ++axis) {
            cssSensors.at(sensor).nHat_B(static_cast<Eigen::Index>(axis)) = cssNHat_B.data[(sensor * 3U) + axis];
        }
        cssSensors.at(sensor).availability = fsw::toDeviceAvailability(cssAvailability.availability[sensor]);
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

bool CssWeightedLeastSquaresAlgorithm_validateConfig(const CssBoresightArray_c* cssNHat_B,
                                                     const CssAvailabilityArray_c* cssAvailability,
                                                     const bool useMeasurementsAsWeights,
                                                     const float sensorUseThresh,
                                                     const float controlPeriod) {
    try {
        (void)configFromC(*cssNHat_B, *cssAvailability, useMeasurementsAsWeights, sensorUseThresh, controlPeriod);
        return true;
    } catch (const fsw::invalid_argument&) {
        return false;
    }
}

CssWeightedLeastSquaresAlgorithmHandle* CssWeightedLeastSquaresAlgorithm_create(
    const CssBoresightArray_c* cssNHat_B,
    const CssAvailabilityArray_c* cssAvailability,
    const bool useMeasurementsAsWeights,
    const float sensorUseThresh,
    const float controlPeriod) {
    return fsw::createHandle<::CssWeightedLeastSquaresAlgorithm, CssWeightedLeastSquaresAlgorithmHandle>(
        configFromC(*cssNHat_B, *cssAvailability, useMeasurementsAsWeights, sensorUseThresh, controlPeriod));
}

void CssWeightedLeastSquaresAlgorithm_destroy(CssWeightedLeastSquaresAlgorithmHandle* self) {
    fsw::deleteHandle<::CssWeightedLeastSquaresAlgorithm>(self);
}

void CssWeightedLeastSquaresAlgorithm_setConfig(CssWeightedLeastSquaresAlgorithmHandle* self,
                                                const CssBoresightArray_c* cssNHat_B,
                                                const CssAvailabilityArray_c* cssAvailability,
                                                const bool useMeasurementsAsWeights,
                                                const float sensorUseThresh,
                                                const float controlPeriod) {
    fsw::fromHandle<::CssWeightedLeastSquaresAlgorithm>(self)->setConfig(
        configFromC(*cssNHat_B, *cssAvailability, useMeasurementsAsWeights, sensorUseThresh, controlPeriod));
}

void CssWeightedLeastSquaresAlgorithm_reInitialize(CssWeightedLeastSquaresAlgorithmHandle* self) {
    fsw::fromHandle<::CssWeightedLeastSquaresAlgorithm>(self)->reInitialize();
}

CssWeightedLeastSquaresOutput_c CssWeightedLeastSquaresAlgorithm_update(CssWeightedLeastSquaresAlgorithmHandle* self,
                                                                        const CssReadingArray_c* cosValues) {
    const CssWeightedLeastSquaresOutput out =
        fsw::fromHandle<::CssWeightedLeastSquaresAlgorithm>(self)->update(cArrayToEigenVector(cosValues->cosValues));
    return outputToC(out);
}
