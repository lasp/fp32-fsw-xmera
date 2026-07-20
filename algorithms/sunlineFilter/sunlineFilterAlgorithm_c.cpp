#include "sunlineFilterAlgorithm_c.h"
#include "sunlineFilterAlgorithm.h"
#include "sunlineFilterTypes.h"

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

SunlineFilterConfig configFromC(const SunlineFilterConfig_c& c) {
    StateMatrix processNoise;
    StateMatrix initialCovariance;
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            processNoise(i, j) = c.processNoise[i][j];
            initialCovariance(i, j) = c.initialCovariance[i][j];
        }
    }

    Eigen::Vector<double, N> stateSeed;
    for (int i = 0; i < N; ++i) {
        stateSeed(i) = c.initialState[i];
    }

    Eigen::Matrix<double, MaxCss, 3> cssNHat;
    Eigen::Vector<double, MaxCss> cssScaleFactor;
    for (int i = 0; i < MaxCss; ++i) {
        cssScaleFactor(i) = c.cssScaleFactor[i];
        for (int j = 0; j < 3; ++j) {
            cssNHat(i, j) = c.cssNHat[i][j];
        }
    }

    return SunlineFilterConfig::create(c.alpha,
                                       c.beta,
                                       processNoise,
                                       SunlineState{stateSeed},
                                       initialCovariance,
                                       c.biasLowerBound,
                                       c.biasUpperBound,
                                       cssNHat,
                                       cssScaleFactor,
                                       c.numberOfCss,
                                       c.sensorThreshold,
                                       c.cssMeasurementNoiseStd,
                                       c.gyroMeasurementNoiseStd);
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

SunlineFilterAlgorithmHandle* SunlineFilterAlgorithm_create(const SunlineFilterConfig_c* config) {
    return reinterpret_cast<SunlineFilterAlgorithmHandle*>(new ::SunlineFilterAlgorithm(configFromC(*config)));
}

void SunlineFilterAlgorithm_destroy(SunlineFilterAlgorithmHandle* self) {
    delete reinterpret_cast<::SunlineFilterAlgorithm*>(self);
}

void SunlineFilterAlgorithm_setConfig(SunlineFilterAlgorithmHandle* self, const SunlineFilterConfig_c* config) {
    reinterpret_cast<::SunlineFilterAlgorithm*>(self)->setConfig(configFromC(*config));
}

SunlineFilterOutput_c SunlineFilterAlgorithm_update(SunlineFilterAlgorithmHandle* self,
                                                    const double currentSeconds,
                                                    const SunlineCssData_c* cssData,
                                                    const SunlineRateData_c* rateData) {
    CssData cssDataCpp{};
    cssDataCpp.timeTag = cssData->timeTag;
    for (int i = 0; i < MaxCss; ++i) {
        cssDataCpp.cosValues(i) = cssData->cosValues[i];
    }

    RateData rateDataCpp{};
    rateDataCpp.timeTag = rateData->timeTag;
    rateDataCpp.rate << rateData->rate[0], rateData->rate[1], rateData->rate[2];

    const SunlineFilterOutput out =
        reinterpret_cast<::SunlineFilterAlgorithm*>(self)->update(currentSeconds, cssDataCpp, rateDataCpp);
    return outputToC(out);
}

void SunlineFilterAlgorithm_reInitializeExceptPersistentStates(SunlineFilterAlgorithmHandle* self) {
    reinterpret_cast<::SunlineFilterAlgorithm*>(self)->reInitializeExceptPersistentStates();
}

void SunlineFilterAlgorithm_reInitialize(SunlineFilterAlgorithmHandle* self) {
    reinterpret_cast<::SunlineFilterAlgorithm*>(self)->reInitialize();
}

SunlineFilterStateOutput_c SunlineFilterAlgorithm_getFilterOutput(const SunlineFilterAlgorithmHandle* self) {
    return filterStateToC(reinterpret_cast<const ::SunlineFilterAlgorithm*>(self)->getFilterOutput());
}

SunlineCssResidualsOutput_c SunlineFilterAlgorithm_getLastCssResiduals(const SunlineFilterAlgorithmHandle* self) {
    return cssResidualsToC(reinterpret_cast<const ::SunlineFilterAlgorithm*>(self)->getLastCssResiduals());
}

SunlineRateResidualsOutput_c SunlineFilterAlgorithm_getLastRateResiduals(const SunlineFilterAlgorithmHandle* self) {
    return rateResidualsToC(reinterpret_cast<const ::SunlineFilterAlgorithm*>(self)->getLastRateResiduals());
}
