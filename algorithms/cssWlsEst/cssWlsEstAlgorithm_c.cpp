#include "cssWlsEstAlgorithm_c.h"

#include "cssWlsEstAlgorithm.h"
#include "cssWlsEstTypes.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/opaqueHandle.h"

#include <Eigen/Core>

static_assert(CSS_WLS_EST_MAX_NUM_CSS == kMaxNumCss, "CSS_WLS_EST_MAX_NUM_CSS must match kMaxNumCss");

namespace {

/*! Build the validated C++ configuration from its C mirror. The boresight rows are copied element by
    element rather than mapped, because the C POD stores them as a two-dimensional array whose row
    layout must not be reinterpreted. */
CssWlsEstConfig configFromC(const CssWlsEstConstellation_c& constellation,
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

    return CssWlsEstConfig::create(cssNHat_B, cssBias, constellation.numCss, useWeights, sensorUseThresh);
}

/*! Convert the algorithm's output struct to its C mirror. */
CssWlsEstOutput_c outputToC(const CssWlsEstOutput& out) {
    CssWlsEstOutput_c result{};
    eigenVectorToCArray(out.sunHeading_B, result.sunHeading_B.data);
    eigenVectorToCArray(out.omega_BN_B, result.omega_BN_B.data);
    eigenVectorToCArray(out.residualStateHeading, result.residualStateHeading.data);
    eigenVectorToCArray(out.postFitResiduals, result.postFitResiduals);
    result.numActiveCss = out.numActiveCss;
    return result;
}

}  // namespace

uint32_t CssWlsEstAlgorithm_getMaxNumCss(void) { return CSS_WLS_EST_MAX_NUM_CSS; }

bool CssWlsEstAlgorithm_validateConfig(const CssWlsEstConstellation_c* constellation,
                                       const bool useWeights,
                                       const float sensorUseThresh) {
    try {
        (void)configFromC(*constellation, useWeights, sensorUseThresh);
        return true;
    } catch (const fsw::invalid_argument&) {
        return false;
    }
}

CssWlsEstAlgorithmHandle* CssWlsEstAlgorithm_create(const CssWlsEstConstellation_c* constellation,
                                                    const bool useWeights,
                                                    const float sensorUseThresh) {
    return fsw::createHandle<::CssWlsEstAlgorithm, CssWlsEstAlgorithmHandle>(
        configFromC(*constellation, useWeights, sensorUseThresh));
}

void CssWlsEstAlgorithm_destroy(CssWlsEstAlgorithmHandle* self) { fsw::deleteHandle<::CssWlsEstAlgorithm>(self); }

void CssWlsEstAlgorithm_setConfig(CssWlsEstAlgorithmHandle* self,
                                  const CssWlsEstConstellation_c* constellation,
                                  const bool useWeights,
                                  const float sensorUseThresh) {
    fsw::fromHandle<::CssWlsEstAlgorithm>(self)->setConfig(configFromC(*constellation, useWeights, sensorUseThresh));
}

void CssWlsEstAlgorithm_reInitialize(CssWlsEstAlgorithmHandle* self) {
    fsw::fromHandle<::CssWlsEstAlgorithm>(self)->reInitialize();
}

CssWlsEstOutput_c CssWlsEstAlgorithm_update(CssWlsEstAlgorithmHandle* self,
                                            const uint64_t callTime,
                                            const CssWlsEstInputs_c* inputs) {
    const CssWlsEstOutput out =
        fsw::fromHandle<::CssWlsEstAlgorithm>(self)->update(callTime, cArrayToEigenVector(inputs->cosValues));
    return outputToC(out);
}
