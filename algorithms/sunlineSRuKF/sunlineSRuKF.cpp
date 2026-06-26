#include "sunlineSRuKF.h"

#include "sunlineSRuKFAlgorithm.h"
#include "utilities/xmeraLifecycleException.h"

#include <architecture/utilities/eigenSupport.h>
#include <architecture/utilities/macroDefinitions.h>

#include <Eigen/Core>

#include <stdexcept>

using filtering::sunlineSRuKF::CssData;
using filtering::sunlineSRuKF::MaxCss;
using filtering::sunlineSRuKF::RateData;
using filtering::sunlineSRuKF::StateMatrix;
using filtering::sunlineSRuKF::SunlineSRuKFAlgorithm;
using filtering::sunlineSRuKF::SunlineSRuKFConfig;
using filtering::sunlineSRuKF::SunlineSRuKFOutput;
using filtering::sunlineSRuKF::SunlineState;

SunlineSRuKF::SunlineSRuKF() {
    constexpr int n = SunlineSRuKFAlgorithm::N;
    this->processNoise = Eigen::MatrixXd::Zero(n, n);
    this->initialState = Eigen::VectorXd::Zero(n);
    this->initialCovariance = Eigen::MatrixXd::Identity(n, n);
}

SunlineSRuKF::~SunlineSRuKF() = default;

/*! Validate message connections, build a validated config from the public properties and the CSS
 *  geometry latched from cssConfigInMsg, and construct the algorithm (its constructor seeds the
 *  filter state and covariance).
 *  @return void
 *  @param currentSimNanos [ns] sim time at which reset was called */
void SunlineSRuKF::reset(uint64_t currentSimNanos) {
    if (!this->navAttInMsg.isLinked()) {
        throw std::invalid_argument("sunlineSRuKF.navAttInMsg wasn't connected.");
    }
    if (!this->cssDataInMsg.isLinked()) {
        throw std::invalid_argument("sunlineSRuKF.cssDataInMsg wasn't connected.");
    }
    if (!this->cssConfigInMsg.isLinked()) {
        throw std::invalid_argument("sunlineSRuKF.cssConfigInMsg wasn't connected.");
    }

    auto const cssConfig = this->cssConfigInMsg();
    int const numCss = static_cast<int>(cssConfig.nCSS);
    Eigen::Matrix<double, MaxCss, 3> nHat = Eigen::Matrix<double, MaxCss, 3>::Zero();
    Eigen::Vector<double, MaxCss> cssScaleFactor = Eigen::Vector<double, MaxCss>::Zero();
    for (int i = 0; i < numCss; ++i) {
        cssScaleFactor(i) = cssConfig.cssVals[i].CBias;
        for (int j = 0; j < 3; ++j) {
            nHat(i, j) = cssConfig.cssVals[i].nHat_B[j];
        }
    }
    this->numberOfCss = numCss;

    constexpr int n = SunlineSRuKFAlgorithm::N;
    auto const config = SunlineSRuKFConfig::create(this->alpha,
                                                   this->beta,
                                                   StateMatrix(this->processNoise),
                                                   SunlineState{Eigen::Vector<double, n>(this->initialState)},
                                                   StateMatrix(this->initialCovariance),
                                                   this->biasLowerBound,
                                                   this->biasUpperBound,
                                                   nHat,
                                                   cssScaleFactor,
                                                   numCss,
                                                   this->sensorThreshold,
                                                   this->cssMeasurementNoiseStd,
                                                   this->gyroMeasurementNoiseStd);
    this->algorithm = std::make_unique<SunlineSRuKFAlgorithm>(config);
    this->lastNavAttTimeTag = 0;
    this->lastCssTimeTag = 0;
}

/*! Clear the filter's internal runtime state; state and covariance are preserved.
 *  @return void */
void SunlineSRuKF::reInitialize() {
    if (!this->algorithm) {
        throw XmeraLifecycleException("SunlineSRuKF reset() has not been called.");
    }
    this->algorithm->reInitialize();
}

/*! reInitialize() and additionally re-seed the filter state and covariance from the configuration.
 *  @return void */
void SunlineSRuKF::reInitializeAll() {
    if (!this->algorithm) {
        throw XmeraLifecycleException("SunlineSRuKF reset() has not been called.");
    }
    this->algorithm->reInitializeAll();
}

/*! Read NavAtt and CSS messages, call algorithm update, and
 *  write the output state and residuals.
 *  @return void
 *  @param currentSimNanos [ns] sim time the filter is advancing to */
void SunlineSRuKF::updateState(uint64_t currentSimNanos) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("SunlineSRuKF reset() has not been called.");
    }

    double const currentSeconds = static_cast<double>(currentSimNanos) * NANO2SEC;

    RateData rateData{};
    CssData cssData{};

    if (auto const navMsgPayload = this->navAttInMsg(); navMsgPayload.timeTag > this->lastNavAttTimeTag) {
        rateData.timeTag = navMsgPayload.timeTag;
        rateData.rate = cArrayToEigenVector(navMsgPayload.omega_BN_B);
        this->lastNavAttTimeTag = navMsgPayload.timeTag;
    }

    if (auto const [timeTag, CosValue] = this->cssDataInMsg(); timeTag > this->lastCssTimeTag) {
        Eigen::Vector<double, MaxCss> sensorMeasurements{};
        for (int i = 0; i < this->numberOfCss; ++i) {
            sensorMeasurements[i] = CosValue[i];
        }
        cssData.timeTag = timeTag;
        cssData.cosValues = sensorMeasurements;
        this->lastCssTimeTag = timeTag;
    }

    SunlineSRuKFOutput const filterOutput = this->algorithm->update(currentSeconds, cssData, rateData);
    this->writeOutputMessages(currentSimNanos, filterOutput);
}

/*! Write the algorithm's output to the four xmera output messages.
 *  @return void
 *  @param currentSimNanos [ns] sim time provided to the outgoing messages
 *  @param filterOutput    [-]  filter data returned by algorithm */
void SunlineSRuKF::writeOutputMessages(uint64_t currentSimNanos, SunlineSRuKFOutput const& filterOutput) {
    NavAttMsgPayload navAttBuf{};
    FilterMsgPayload filterBuf{};
    FilterResidualsMsgPayload gyroResBuf{};
    FilterResidualsMsgPayload cssResBuf{};

    double const timeTag = static_cast<double>(currentSimNanos) * NANO2SEC;

    eigenMatrixXToCArray(filterOutput.filterState.state.head<3>().eval(), navAttBuf.vehSunPntBdy);

    filterBuf.timeTag = timeTag;
    filterBuf.numberOfStates = SunlineSRuKFAlgorithm::N;
    eigenMatrixXToCArray(filterOutput.filterState.state, filterBuf.state);
    eigenMatrixXToCArray(filterOutput.filterState.covariance, filterBuf.covar);

    if (filterOutput.cssResiduals.valid) {
        cssResBuf.timeTag = timeTag;
        cssResBuf.valid = true;
        cssResBuf.numberOfObservations = 1;
        cssResBuf.sizeOfObservations = filterOutput.cssResiduals.numberOfActiveCss;
        eigenMatrixXToCArray(filterOutput.cssResiduals.observation, cssResBuf.observation);
        eigenMatrixXToCArray(filterOutput.cssResiduals.preFit, cssResBuf.preFits);
        eigenMatrixXToCArray(filterOutput.cssResiduals.postFit, cssResBuf.postFits);
    }
    if (filterOutput.rateResiduals.valid) {
        gyroResBuf.timeTag = timeTag;
        gyroResBuf.valid = true;
        gyroResBuf.numberOfObservations = 1;
        gyroResBuf.sizeOfObservations = 3;
        eigenMatrixXToCArray(filterOutput.rateResiduals.observation, gyroResBuf.observation);
        eigenMatrixXToCArray(filterOutput.rateResiduals.preFit, gyroResBuf.preFits);
        eigenMatrixXToCArray(filterOutput.rateResiduals.postFit, gyroResBuf.postFits);
    }

    this->navAttOutMsg.write(navAttBuf, this->moduleID, currentSimNanos);
    this->filterOutMsg.write(filterBuf, this->moduleID, currentSimNanos);
    this->filterGyroResOutMsg.write(gyroResBuf, this->moduleID, currentSimNanos);
    this->filterCssResOutMsg.write(cssResBuf, this->moduleID, currentSimNanos);
}
