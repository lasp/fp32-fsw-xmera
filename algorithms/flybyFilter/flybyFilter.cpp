#include "flybyFilter.h"

#include "flybyFilterAlgorithm.h"
#include "utilities/xmera/xmeraLifecycleException.h"

#include <architecture/utilities/eigenSupport.h>
#include <utilities/fsw/timeConstants.h>

#include <Eigen/Core>

#include <stdexcept>

using filtering::flybyFilter::FlybyFilterAlgorithm;
using filtering::flybyFilter::FlybyFilterConfig;
using filtering::flybyFilter::FlybyFilterOutput;
using filtering::flybyFilter::FlybyState;
using filtering::flybyFilter::HeadingData;
using filtering::flybyFilter::StateMatrix;

FlybyFilter::FlybyFilter() {
    constexpr int n = FlybyFilterAlgorithm::N;
    this->processNoise = Eigen::MatrixXd::Zero(n, n);
    this->initialState = Eigen::VectorXd::Zero(n);
    this->initialCovariance = Eigen::MatrixXd::Identity(n, n);
}

FlybyFilter::~FlybyFilter() = default;

/*! Validate message connections, convert the SI public properties into internal (km) units, build a
 *  validated config, and construct the algorithm (its constructor seeds the filter state/covariance).
 *  @return void
 *  @param currentSimNanos [ns] sim time at which reset was called */
void FlybyFilter::reset(uint64_t /*currentSimNanos*/) {
    if (!this->opNavHeadingMsg.isLinked()) {
        throw std::invalid_argument("flybyFilter.opNavHeadingMsg wasn't connected.");
    }

    constexpr int n = FlybyFilterAlgorithm::N;
    double const uc = this->unitConversion;
    double const uc2 = uc * uc;
    double const uc3 = uc2 * uc;

    // SI -> internal: state x uc, covariance / process-noise x uc^2, mu x uc^3.
    Eigen::Vector<double, n> const internalStateVec = Eigen::Vector<double, n>(this->initialState) * uc;
    FlybyState const internalState{internalStateVec};
    StateMatrix const internalCovariance = StateMatrix(this->initialCovariance) * uc2;
    StateMatrix const internalProcessNoise = StateMatrix(this->processNoise) * uc2;

    auto const config = FlybyFilterConfig::create(this->alpha,
                                                  this->beta,
                                                  this->mu * uc3,
                                                  internalProcessNoise,
                                                  internalState,
                                                  internalCovariance,
                                                  this->headingMeasurementNoiseStd,
                                                  this->outlierNSigma);
    this->algorithm = std::make_unique<FlybyFilterAlgorithm>(config);
    this->lastHeadingTimeTag = 0;
}

/*! Clear the filter's internal runtime state; state and covariance are preserved.
 *  @return void */
void FlybyFilter::reInitializeExceptPersistentStates() {
    if (!this->algorithm) {
        throw XmeraLifecycleException("FlybyFilter reset() has not been called.");
    }
    this->algorithm->reInitializeExceptPersistentStates();
}

/*! reInitializeExceptPersistentStates() and additionally re-seed the filter state and covariance from
 *  the configuration.
 *  @return void */
void FlybyFilter::reInitialize() {
    if (!this->algorithm) {
        throw XmeraLifecycleException("FlybyFilter reset() has not been called.");
    }
    this->algorithm->reInitialize();
}

/*! Read the heading message, call the algorithm update, and write the output state and residuals.
 *  @return void
 *  @param currentSimNanos [ns] sim time the filter is advancing to */
void FlybyFilter::updateState(uint64_t currentSimNanos) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("FlybyFilter reset() has not been called.");
    }

    double const currentSeconds = static_cast<double>(currentSimNanos) * kNano2Sec;

    HeadingData headingData{};
    if (auto const payload = this->opNavHeadingMsg(); payload.valid && payload.timeTag > this->lastHeadingTimeTag) {
        headingData.timeTag = payload.timeTag;
        headingData.rhat_BN_N = cArrayToEigenVector(payload.rhat_BN_N);
        this->lastHeadingTimeTag = payload.timeTag;
    }

    FlybyFilterOutput const filterOutput = this->algorithm->update(currentSeconds, headingData);
    this->writeOutputMessages(currentSimNanos, filterOutput);
}

/*! Write the algorithm's output to the three xmera output messages, converting internal (km) units
 *  back to SI.
 *  @return void
 *  @param currentSimNanos [ns] sim time provided to the outgoing messages
 *  @param filterOutput    [-]  filter data returned by the algorithm */
void FlybyFilter::writeOutputMessages(uint64_t currentSimNanos, FlybyFilterOutput const& filterOutput) {
    NavTransMsgPayload navTransBuf{};
    FilterMsgPayload filterBuf{};
    FilterResidualsMsgPayload resBuf{};

    double const timeTag = static_cast<double>(currentSimNanos) * kNano2Sec;
    double const invUc = 1.0 / this->unitConversion;
    double const invUc2 = invUc * invUc;

    // internal -> SI: state x (1/uc), covariance x (1/uc^2).
    Eigen::Matrix<double, FlybyFilterAlgorithm::N, 1> const stateSI = filterOutput.filterState.state * invUc;
    Eigen::Matrix<double, FlybyFilterAlgorithm::N, FlybyFilterAlgorithm::N> const covarSI =
        filterOutput.filterState.covariance * invUc2;

    navTransBuf.timeTag = timeTag;
    eigenMatrixXToCArray(stateSI.head<3>().eval(), navTransBuf.r_BN_N);
    eigenMatrixXToCArray(stateSI.segment<3>(3).eval(), navTransBuf.v_BN_N);

    filterBuf.timeTag = timeTag;
    filterBuf.numberOfStates = FlybyFilterAlgorithm::N;
    eigenMatrixXToCArray(stateSI, filterBuf.state);
    eigenMatrixXToCArray(covarSI, filterBuf.covar);

    if (filterOutput.headingResiduals.valid) {
        resBuf.timeTag = timeTag;
        resBuf.valid = true;
        resBuf.numberOfObservations = 1;
        resBuf.sizeOfObservations = 3;
        eigenMatrixXToCArray(filterOutput.headingResiduals.observation, resBuf.observation);
        eigenMatrixXToCArray(filterOutput.headingResiduals.preFit, resBuf.preFits);
        eigenMatrixXToCArray(filterOutput.headingResiduals.postFit, resBuf.postFits);
    }

    this->navTransOutMsg.write(navTransBuf, this->moduleID, currentSimNanos);
    this->filterOutMsg.write(filterBuf, this->moduleID, currentSimNanos);
    this->filterResOutMsg.write(resBuf, this->moduleID, currentSimNanos);
}
