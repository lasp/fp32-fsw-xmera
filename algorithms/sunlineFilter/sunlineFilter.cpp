#include "sunlineFilter.h"

#include "sunlineFilterAlgorithm.h"
#include "utilities/xmera/xmeraLifecycleException.h"
#include <utilities/fsw/eigenSupport.h>
#include <stdexcept>
#include <utilities/fsw/timeConversion.hpp>

#include <Eigen/Core>

using filtering::sunlineFilter::CssData;
using filtering::sunlineFilter::MaxCss;
using filtering::sunlineFilter::RateData;
using filtering::sunlineFilter::StateMatrix;
using filtering::sunlineFilter::SunlineFilterAlgorithm;
using filtering::sunlineFilter::SunlineFilterConfig;
using filtering::sunlineFilter::SunlineFilterOutput;
using filtering::sunlineFilter::SunlineState;

SunlineFilter::SunlineFilter()
    : processNoise(StateMatrix::Zero()),
      initialState(StateColumn::Zero()),
      initialCovariance(StateMatrix::Identity()) {}

SunlineFilter::~SunlineFilter() = default;

/*! Validate message connections, build a validated config from the public properties and the CSS
 *  geometry latched from cssConfigInMsg, and construct the algorithm (its constructor seeds the
 *  filter state and covariance).
 *  @return void
 *  @param currentSimNanos [ns] sim time at which reset was called */
void SunlineFilter::reset(uint64_t currentSimNanos) {
    if (!this->navAttInMsg.isLinked()) {
        throw std::invalid_argument("sunlineFilter.navAttInMsg wasn't connected.");
    }
    if (!this->cssDataInMsg.isLinked()) {
        throw std::invalid_argument("sunlineFilter.cssDataInMsg wasn't connected.");
    }
    if (!this->cssConfigInMsg.isLinked()) {
        throw std::invalid_argument("sunlineFilter.cssConfigInMsg wasn't connected.");
    }

    const auto [nCSS, cssVals] = this->cssConfigInMsg();
    if (nCSS != MaxCss) {
        throw std::invalid_argument(
            "sunlineFilter: the number of CSS in the cssConfig "
            "does not equal the compile time MaxCss.");
    }

    Eigen::Matrix<float, MaxCss, 3> nHat = Eigen::Matrix<float, MaxCss, 3>::Zero();
    Eigen::Vector<float, MaxCss> cssScaleFactor = Eigen::Vector<float, MaxCss>::Zero();

    for (uint32_t i = 0; i < MaxCss; ++i) {
        cssScaleFactor(i) = cssVals[i].CBias;
        for (int j = 0; j < 3; ++j) {
            nHat(i, j) = cssVals[i].nHat_B[j];
        }
    }

    auto const config = SunlineFilterConfig::create(this->alpha,
                                                    this->beta,
                                                    this->processNoise,
                                                    SunlineState{this->initialState},
                                                    this->initialCovariance,
                                                    this->biasLowerBound,
                                                    this->biasUpperBound,
                                                    nHat,
                                                    cssScaleFactor,
                                                    this->sensorThreshold,
                                                    this->cssMeasurementNoiseStd,
                                                    this->gyroMeasurementNoiseStd,
                                                    this->measurementDelay);
    this->algorithm = std::make_unique<SunlineFilterAlgorithm>(config);
    this->lastNavAttTimeTag = 0;
    this->lastCssTimeTag = 0;
}

/*! Clear the filter's internal runtime state; state and covariance are preserved.
 *  @return void */
void SunlineFilter::reInitializeExceptPersistentStates() {
    if (!this->algorithm) {
        throw XmeraLifecycleException("SunlineFilter reset() has not been called.");
    }
    this->algorithm->reInitializeExceptPersistentStates();
}

/*! reInitializeExceptPersistentStates() and additionally re-seed the filter state and covariance from the
 * configuration.
 *  @return void */
void SunlineFilter::reInitialize() {
    if (!this->algorithm) {
        throw XmeraLifecycleException("SunlineFilter reset() has not been called.");
    }
    this->algorithm->reInitialize();
}

/*! Read NavAtt and CSS messages, call algorithm update, and
 *  write the output state and residuals.
 *  @return void
 *  @param currentSimNanos [ns] sim time the filter is advancing to */
void SunlineFilter::updateState(uint64_t currentSimNanos) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("SunlineFilter reset() has not been called.");
    }

    double const currentSeconds = fsw::nanosToSeconds(currentSimNanos);

    RateData rateData{};
    CssData cssData{};

    if (auto const navMsgPayload = this->navAttInMsg(); navMsgPayload.timeTag > this->lastNavAttTimeTag) {
        rateData.timeTag = navMsgPayload.timeTag;
        rateData.rate = cArrayToEigenVector(navMsgPayload.omega_BN_B);
        this->lastNavAttTimeTag = navMsgPayload.timeTag;
    }

    if (auto const [timeTag, CosValue] = this->cssDataInMsg(); timeTag > this->lastCssTimeTag) {
        Eigen::Vector<float, MaxCss> sensorMeasurements{};
        for (uint32_t i = 0; i < MaxCss; ++i) {
            sensorMeasurements[i] = CosValue[i];
        }
        cssData.timeTag = timeTag;
        cssData.cosValues = sensorMeasurements;
        this->lastCssTimeTag = timeTag;
    }

    SunlineFilterOutput const filterOutput = this->algorithm->update(currentSeconds, cssData, rateData);
    this->writeOutputMessages(currentSimNanos, filterOutput);
}

/*! Write the algorithm's output to the four xmera output messages.
 *  @return void
 *  @param currentSimNanos [ns] sim time provided to the outgoing messages
 *  @param filterOutput    [-]  filter data returned by algorithm */
void SunlineFilter::writeOutputMessages(uint64_t currentSimNanos, SunlineFilterOutput const& filterOutput) {
    NavAttMsgF32Payload navAttBuf{};
    FilterMsgF32Payload filterBuf{};
    FilterResidualsMsgF32Payload gyroResBuf{};
    FilterResidualsMsgF32Payload cssResBuf{};

    double const timeTag = fsw::nanosToSeconds(currentSimNanos);

    eigenVectorToCArray(filterOutput.filterState.state.head<3>().eval(), navAttBuf.vehSunPntBdy);

    filterBuf.timeTag = timeTag;
    filterBuf.numberOfStates = SunlineFilterAlgorithm::N;
    eigenMatrixXToCArray(filterOutput.filterState.state.cast<double>().eval(), filterBuf.state);
    eigenMatrixXToCArray(filterOutput.filterState.covariance.cast<double>().eval(), filterBuf.covar);

    if (filterOutput.cssResiduals.valid) {
        cssResBuf.timeTag = timeTag;
        cssResBuf.valid = true;
        cssResBuf.numberOfObservations = 1;
        cssResBuf.sizeOfObservations = filterOutput.cssResiduals.numberOfActiveCss;
        eigenMatrixXToCArray(filterOutput.cssResiduals.observation.cast<double>().eval(), cssResBuf.observation);
        eigenMatrixXToCArray(filterOutput.cssResiduals.preFit.cast<double>().eval(), cssResBuf.preFits);
        eigenMatrixXToCArray(filterOutput.cssResiduals.postFit.cast<double>().eval(), cssResBuf.postFits);
    }
    if (filterOutput.rateResiduals.valid) {
        gyroResBuf.timeTag = timeTag;
        gyroResBuf.valid = true;
        gyroResBuf.numberOfObservations = 1;
        gyroResBuf.sizeOfObservations = 3;
        eigenMatrixXToCArray(filterOutput.rateResiduals.observation.cast<double>().eval(), gyroResBuf.observation);
        eigenMatrixXToCArray(filterOutput.rateResiduals.preFit.cast<double>().eval(), gyroResBuf.preFits);
        eigenMatrixXToCArray(filterOutput.rateResiduals.postFit.cast<double>().eval(), gyroResBuf.postFits);
    }

    this->navAttOutMsg.write(navAttBuf, this->moduleID, currentSimNanos);
    this->filterOutMsg.write(filterBuf, this->moduleID, currentSimNanos);
    this->filterGyroResOutMsg.write(gyroResBuf, this->moduleID, currentSimNanos);
    this->filterCssResOutMsg.write(cssResBuf, this->moduleID, currentSimNanos);
}
