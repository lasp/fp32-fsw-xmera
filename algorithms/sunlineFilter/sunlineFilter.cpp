#include "sunlineFilter.h"

#include "sunlineFilterAlgorithm.h"
#include "utilities/xmera/xmeraLifecycleException.h"
#include <utilities/fsw/eigenSupport.h>
#include <stdexcept>

#include <architecture/utilities/macroDefinitions.h>

#include <Eigen/Core>

using filtering::sunlineFilter::CssData;
using filtering::sunlineFilter::MaxCss;
using filtering::sunlineFilter::RateData;
using filtering::sunlineFilter::StateMatrix;
using filtering::sunlineFilter::SunlineFilterAlgorithm;
using filtering::sunlineFilter::SunlineFilterConfig;
using filtering::sunlineFilter::SunlineFilterOutput;
using filtering::sunlineFilter::SunlineState;

SunlineFilter::SunlineFilter() {
    constexpr int n = SunlineFilterAlgorithm::N;
    this->processNoise = Eigen::MatrixXd::Zero(n, n);
    this->initialState = Eigen::VectorXd::Zero(n);
    this->initialCovariance = Eigen::MatrixXd::Identity(n, n);
}

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
    if (this->dt <= 0.0) {
        throw std::invalid_argument("sunlineFilter.dt must be set to a positive time step.");
    }

    auto const cssConfig = this->cssConfigInMsg();
    uint32_t const numCss = cssConfig.nCSS;
    Eigen::Matrix<double, MaxCss, 3> nHat = Eigen::Matrix<double, MaxCss, 3>::Zero();
    Eigen::Vector<double, MaxCss> cssScaleFactor = Eigen::Vector<double, MaxCss>::Zero();
    for (uint32_t i = 0; i < numCss; ++i) {
        cssScaleFactor(i) = cssConfig.cssVals[i].CBias;
        for (int j = 0; j < 3; ++j) {
            nHat(i, j) = cssConfig.cssVals[i].nHat_B[j];
        }
    }
    this->numberOfCss = numCss;

    constexpr int n = SunlineFilterAlgorithm::N;
    auto const config = SunlineFilterConfig::create(this->alpha,
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
    this->algorithm = std::make_unique<SunlineFilterAlgorithm>(config);
    this->algorithm->setDt(this->dt);
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

/*! Pass-through to the algorithm's filter time step. The value is retained on the adapter so it can be set
 *  before reset() (it seeds the algorithm when constructed) and forwarded live once the algorithm exists.
 *  @return void
 *  @param newDt [s] filter time step */
void SunlineFilter::setDt(double newDt) {
    this->dt = newDt;
    if (this->algorithm) {
        this->algorithm->setDt(newDt);
    }
}

/*! @return the filter time step, from the live algorithm once constructed, otherwise the pending value */
double SunlineFilter::getDt() const { return this->algorithm ? this->algorithm->getDt() : this->dt; }

/*! Read NavAtt and CSS messages, call algorithm update, and
 *  write the output state and residuals.
 *  @return void
 *  @param currentSimNanos [ns] sim time the filter is advancing to */
void SunlineFilter::updateState(uint64_t currentSimNanos) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("SunlineFilter reset() has not been called.");
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
        for (uint32_t i = 0; i < this->numberOfCss; ++i) {
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
    NavAttMsgPayload navAttBuf{};
    FilterMsgPayload filterBuf{};
    FilterResidualsMsgPayload gyroResBuf{};
    FilterResidualsMsgPayload cssResBuf{};

    double const timeTag = static_cast<double>(currentSimNanos) * NANO2SEC;

    eigenMatrixXToCArray(filterOutput.filterState.state.head<3>().eval(), navAttBuf.vehSunPntBdy);

    filterBuf.timeTag = timeTag;
    filterBuf.numberOfStates = SunlineFilterAlgorithm::N;
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
