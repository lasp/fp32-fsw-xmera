#include "sunlineFilterAlgorithm_c.h"
#include "sunlineFilterAlgorithm.h"
#include "sunlineFilterTypes.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/opaqueHandle.h"

#include <Eigen/Core>

using filtering::sunlineFilter::CssData;
using filtering::sunlineFilter::CssResidualsOutput;
using filtering::sunlineFilter::FilterStateOutput;
using filtering::sunlineFilter::MaxCss;
using filtering::sunlineFilter::RateData;
using filtering::sunlineFilter::RateResidualsOutput;
using filtering::sunlineFilter::StateMatrix;
using filtering::sunlineFilter::SunlineFilterAlgorithm;
using filtering::sunlineFilter::SunlineFilterConfig;
using filtering::sunlineFilter::SunlineFilterOutput;
using filtering::sunlineFilter::SunlineState;

namespace {

constexpr int N = SunlineFilterAlgorithm::N;

SunlineFilterConfig configFromC(const double alpha,
                                const double beta,
                                const SunlineFilterStateMatrix_c& processNoiseC,
                                const SunlineFilterStateVector_c& initialStateC,
                                const SunlineFilterStateMatrix_c& initialCovarianceC,
                                const double biasLowerBound,
                                const double biasUpperBound,
                                const SunlineFilterCssMatrix_c& cssNHatC,
                                const SunlineFilterCssVector_c& cssScaleFactorC,
                                const double sensorThreshold,
                                const double cssMeasurementNoiseStd,
                                const double gyroMeasurementNoiseStd,
                                const double measurementDelay) {
    StateMatrix processNoise;
    StateMatrix initialCovariance;
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            processNoise(i, j) = static_cast<float>(processNoiseC.data[(i * SUNLINE_FILTER_NUM_STATES) + j]);
            initialCovariance(i, j) = static_cast<float>(initialCovarianceC.data[(i * SUNLINE_FILTER_NUM_STATES) + j]);
        }
    }

    Eigen::Vector<float, N> stateSeed;
    for (int i = 0; i < N; ++i) {
        stateSeed(i) = static_cast<float>(initialStateC.data[i]);
    }

    Eigen::Matrix<float, MaxCss, 3> cssNHat;
    Eigen::Vector<float, MaxCss> cssScaleFactor;
    for (int i = 0; i < MaxCss; ++i) {
        cssScaleFactor(i) = static_cast<float>(cssScaleFactorC.data[i]);
        for (int j = 0; j < 3; ++j) {
            cssNHat(i, j) = static_cast<float>(cssNHatC.data[(i * 3U) + j]);
        }
    }

    return SunlineFilterConfig::create(static_cast<float>(alpha),
                                       static_cast<float>(beta),
                                       processNoise,
                                       SunlineState{stateSeed},
                                       initialCovariance,
                                       static_cast<float>(biasLowerBound),
                                       static_cast<float>(biasUpperBound),
                                       cssNHat,
                                       cssScaleFactor,
                                       static_cast<float>(sensorThreshold),
                                       static_cast<float>(cssMeasurementNoiseStd),
                                       static_cast<float>(gyroMeasurementNoiseStd),
                                       static_cast<float>(measurementDelay));
}

SunlineFilterStateOutput_c filterStateToC(const FilterStateOutput& in) {
    SunlineFilterStateOutput_c out{};
    for (int i = 0; i < N; ++i) {
        out.state[i] = in.state(i);
        for (int j = 0; j < N; ++j) {
            out.covariance[i][j] = in.covariance(i, j);
        }
    }
    return out;
}

SunlineCssResidualsOutput_c cssResidualsToC(const CssResidualsOutput& in) {
    SunlineCssResidualsOutput_c out{};
    out.valid = in.valid;
    out.numberOfActiveCss = in.numberOfActiveCss;
    for (int i = 0; i < MaxCss; ++i) {
        out.observation[i] = in.observation(i);
        out.preFit[i] = in.preFit(i);
        out.postFit[i] = in.postFit(i);
    }
    return out;
}

SunlineRateResidualsOutput_c rateResidualsToC(const RateResidualsOutput& in) {
    SunlineRateResidualsOutput_c out{};
    out.valid = in.valid;
    for (int i = 0; i < 3; ++i) {
        out.observation[i] = in.observation(i);
        out.preFit[i] = in.preFit(i);
        out.postFit[i] = in.postFit(i);
    }
    return out;
}

SunlineFilterOutput_c outputToC(const SunlineFilterOutput& out) {
    SunlineFilterOutput_c result{};
    result.filterState = filterStateToC(out.filterState);
    result.cssResiduals = cssResidualsToC(out.cssResiduals);
    result.rateResiduals = rateResidualsToC(out.rateResiduals);
    return result;
}

}  // namespace

uint32_t SunlineFilterAlgorithm_getMaxCss(void) { return SUNLINE_FILTER_MAX_CSS; }

uint32_t SunlineFilterAlgorithm_getNumStates(void) { return SUNLINE_FILTER_NUM_STATES; }

bool SunlineFilterAlgorithm_validateConfig(double alpha,
                                           double beta,
                                           const SunlineFilterStateMatrix_c* processNoise,
                                           const SunlineFilterStateVector_c* initialState,
                                           const SunlineFilterStateMatrix_c* initialCovariance,
                                           double biasLowerBound,
                                           double biasUpperBound,
                                           const SunlineFilterCssMatrix_c* cssNHat,
                                           const SunlineFilterCssVector_c* cssScaleFactor,
                                           double sensorThreshold,
                                           double cssMeasurementNoiseStd,
                                           double gyroMeasurementNoiseStd,
                                           double measurementDelay) {
    // Build the config through the same path create() uses: success means valid, a throw means
    // invalid. Sharing configFromC keeps the predicate from drifting from what create() accepts.
    try {
        (void)configFromC(alpha,
                          beta,
                          *processNoise,
                          *initialState,
                          *initialCovariance,
                          biasLowerBound,
                          biasUpperBound,
                          *cssNHat,
                          *cssScaleFactor,
                          sensorThreshold,
                          cssMeasurementNoiseStd,
                          gyroMeasurementNoiseStd,
                          measurementDelay);
        return true;
    } catch (const fsw::invalid_argument&) {
        return false;
    }
}

SunlineFilterAlgorithmHandle* SunlineFilterAlgorithm_create(double alpha,
                                                            double beta,
                                                            const SunlineFilterStateMatrix_c* processNoise,
                                                            const SunlineFilterStateVector_c* initialState,
                                                            const SunlineFilterStateMatrix_c* initialCovariance,
                                                            double biasLowerBound,
                                                            double biasUpperBound,
                                                            const SunlineFilterCssMatrix_c* cssNHat,
                                                            const SunlineFilterCssVector_c* cssScaleFactor,
                                                            double sensorThreshold,
                                                            double cssMeasurementNoiseStd,
                                                            double gyroMeasurementNoiseStd,
                                                            double measurementDelay) {
    return fsw::createHandle<::SunlineFilterAlgorithm, SunlineFilterAlgorithmHandle>(
        configFromC(alpha,
                    beta,
                    *processNoise,
                    *initialState,
                    *initialCovariance,
                    biasLowerBound,
                    biasUpperBound,
                    *cssNHat,
                    *cssScaleFactor,
                    sensorThreshold,
                    cssMeasurementNoiseStd,
                    gyroMeasurementNoiseStd,
                    measurementDelay));
}

void SunlineFilterAlgorithm_destroy(SunlineFilterAlgorithmHandle* self) {
    fsw::deleteHandle<::SunlineFilterAlgorithm>(self);
}

void SunlineFilterAlgorithm_setConfig(SunlineFilterAlgorithmHandle* self,
                                      double alpha,
                                      double beta,
                                      const SunlineFilterStateMatrix_c* processNoise,
                                      const SunlineFilterStateVector_c* initialState,
                                      const SunlineFilterStateMatrix_c* initialCovariance,
                                      double biasLowerBound,
                                      double biasUpperBound,
                                      const SunlineFilterCssMatrix_c* cssNHat,
                                      const SunlineFilterCssVector_c* cssScaleFactor,
                                      double sensorThreshold,
                                      double cssMeasurementNoiseStd,
                                      double gyroMeasurementNoiseStd,
                                      double measurementDelay) {
    fsw::fromHandle<::SunlineFilterAlgorithm>(self)->setConfig(configFromC(alpha,
                                                                           beta,
                                                                           *processNoise,
                                                                           *initialState,
                                                                           *initialCovariance,
                                                                           biasLowerBound,
                                                                           biasUpperBound,
                                                                           *cssNHat,
                                                                           *cssScaleFactor,
                                                                           sensorThreshold,
                                                                           cssMeasurementNoiseStd,
                                                                           gyroMeasurementNoiseStd,
                                                                           measurementDelay));
}

SunlineFilterOutput_c SunlineFilterAlgorithm_update(SunlineFilterAlgorithmHandle* self,
                                                    const double currentSeconds,
                                                    const SunlineCssData_c* cssData,
                                                    const SunlineRateData_c* rateData) {
    CssData cssDataCpp{};
    cssDataCpp.timeTag = cssData->timeTag;
    for (int i = 0; i < MaxCss; ++i) {
        cssDataCpp.cosValues(i) = static_cast<float>(cssData->cosValues[i]);
    }

    RateData rateDataCpp{};
    rateDataCpp.timeTag = rateData->timeTag;
    rateDataCpp.rate << static_cast<float>(rateData->rate[0]), static_cast<float>(rateData->rate[1]),
        static_cast<float>(rateData->rate[2]);

    const SunlineFilterOutput out =
        fsw::fromHandle<::SunlineFilterAlgorithm>(self)->update(currentSeconds, cssDataCpp, rateDataCpp);
    return outputToC(out);
}

void SunlineFilterAlgorithm_reInitializeExceptPersistentStates(SunlineFilterAlgorithmHandle* self) {
    fsw::fromHandle<::SunlineFilterAlgorithm>(self)->reInitializeExceptPersistentStates();
}

void SunlineFilterAlgorithm_reInitialize(SunlineFilterAlgorithmHandle* self) {
    fsw::fromHandle<::SunlineFilterAlgorithm>(self)->reInitialize();
}

SunlineFilterStateOutput_c SunlineFilterAlgorithm_getFilterOutput(const SunlineFilterAlgorithmHandle* self) {
    return filterStateToC(fsw::fromHandle<const ::SunlineFilterAlgorithm>(self)->getFilterOutput());
}

SunlineCssResidualsOutput_c SunlineFilterAlgorithm_getLastCssResiduals(const SunlineFilterAlgorithmHandle* self) {
    return cssResidualsToC(fsw::fromHandle<const ::SunlineFilterAlgorithm>(self)->getLastCssResiduals());
}

SunlineRateResidualsOutput_c SunlineFilterAlgorithm_getLastRateResiduals(const SunlineFilterAlgorithmHandle* self) {
    return rateResidualsToC(fsw::fromHandle<const ::SunlineFilterAlgorithm>(self)->getLastRateResiduals());
}
