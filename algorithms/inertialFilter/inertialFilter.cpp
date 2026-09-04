#include "inertialFilter.h"

#include "inertialFilterAlgorithm.h"
#include "utilities/xmera/xmeraLifecycleException.h"

#include <architecture/utilities/macroDefinitions.h>
#include <utilities/fsw/eigenSupport.h>

#include <Eigen/Core>

#include <stdexcept>

using filtering::inertialFilter::InertialFilterAlgorithm;
using filtering::inertialFilter::InertialFilterConfig;
using filtering::inertialFilter::InertialFilterOutput;
using filtering::inertialFilter::InertialState;
using filtering::inertialFilter::RateData;
using filtering::inertialFilter::StateMatrix;
using filtering::inertialFilter::StAttData;

InertialFilter::InertialFilter() {
    constexpr int n = InertialFilterAlgorithm::N;
    this->processNoise = Eigen::MatrixXd::Zero(n, n);
    this->initialState = Eigen::VectorXd::Zero(n);
    this->initialCovariance = Eigen::MatrixXd::Identity(n, n);
}

InertialFilter::~InertialFilter() = default;

/*! Validate message connections, build a validated config from the public properties, and construct
 *  the algorithm (its constructor seeds the filter state and covariance).
 *  @return void
 *  @param currentSimNanos [ns] sim time at which reset was called */
void InertialFilter::reset(uint64_t /*currentSimNanos*/) {
    if (!this->stAttInMsg.isLinked()) {
        throw std::invalid_argument("inertialFilter.stAttInMsg wasn't connected.");
    }

    constexpr int n = InertialFilterAlgorithm::N;
    auto const config = InertialFilterConfig::create(this->alpha,
                                                     this->beta,
                                                     StateMatrix(this->processNoise),
                                                     InertialState{Eigen::Vector<double, n>(this->initialState)},
                                                     StateMatrix(this->initialCovariance),
                                                     this->stMeasurementNoiseStd,
                                                     this->gyroMeasurementNoiseStd,
                                                     this->outlierNSigma);
    this->algorithm = std::make_unique<InertialFilterAlgorithm>(config);
    this->lastStTimeTag = 0;
}

/*! Clear the filter's internal runtime state; state and covariance are preserved.
 *  @return void */
void InertialFilter::reInitializeExceptPersistentStates() {
    if (!this->algorithm) {
        throw XmeraLifecycleException("InertialFilter reset() has not been called.");
    }
    this->algorithm->reInitializeExceptPersistentStates();
}

/*! reInitializeExceptPersistentStates() and additionally re-seed the filter state and covariance from the
 * configuration.
 *  @return void */
void InertialFilter::reInitialize() {
    if (!this->algorithm) {
        throw XmeraLifecycleException("InertialFilter reset() has not been called.");
    }
    this->algorithm->reInitialize();
}

/*! Read star-tracker and gyro messages, call algorithm update, and write the output state and
 *  residuals.
 *  @return void
 *  @param currentSimNanos [ns] sim time the filter is advancing to */
void InertialFilter::updateState(uint64_t currentSimNanos) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("InertialFilter reset() has not been called.");
    }

    double const currentSeconds = static_cast<double>(currentSimNanos) * NANO2SEC;

    StAttData stAttData{};
    RateData rateData{};

    if (auto const stPayload = this->stAttInMsg(); stPayload.timeTag > this->lastStTimeTag) {
        stAttData.timeTag = stPayload.timeTag;
        stAttData.sigma_BN = cArrayToEigenVector(stPayload.MRP_BdyInrtl);
        this->lastStTimeTag = stPayload.timeTag;
    }

    if (this->gyrBuffInMsg.isLinked() && this->gyrBuffInMsg.isWritten()) {
        auto const [accPkts] = this->gyrBuffInMsg();
        rateData.timeTag = currentSeconds;
        rateData.rate = cArrayToEigenVector(accPkts[0].gyro_B);
    }

    InertialFilterOutput const filterOutput = this->algorithm->update(currentSeconds, stAttData, rateData);
    this->writeOutputMessages(currentSimNanos, filterOutput);
}

/*! Write the algorithm's output to the four xmera output messages.
 *  @return void
 *  @param currentSimNanos [ns] sim time provided to the outgoing messages
 *  @param filterOutput    [-]  filter data returned by algorithm */
void InertialFilter::writeOutputMessages(uint64_t currentSimNanos, InertialFilterOutput const& filterOutput) {
    NavAttMsgPayload navAttBuf{};
    FilterMsgPayload filterBuf{};
    FilterResidualsMsgPayload stResBuf{};
    FilterResidualsMsgPayload gyroResBuf{};

    double const timeTag = static_cast<double>(currentSimNanos) * NANO2SEC;

    navAttBuf.timeTag = timeTag;
    eigenMatrixXToCArray(filterOutput.filterState.state.head<3>().eval(), navAttBuf.sigma_BN);
    eigenMatrixXToCArray(filterOutput.filterState.state.segment<3>(3).eval(), navAttBuf.omega_BN_B);

    filterBuf.timeTag = timeTag;
    filterBuf.numberOfStates = InertialFilterAlgorithm::N;
    eigenMatrixXToCArray(filterOutput.filterState.state, filterBuf.state);
    eigenMatrixXToCArray(filterOutput.filterState.covariance, filterBuf.covar);

    if (filterOutput.stAttResiduals.valid) {
        stResBuf.timeTag = timeTag;
        stResBuf.valid = true;
        stResBuf.numberOfObservations = 1;
        stResBuf.sizeOfObservations = 3;
        eigenMatrixXToCArray(filterOutput.stAttResiduals.observation, stResBuf.observation);
        eigenMatrixXToCArray(filterOutput.stAttResiduals.preFit, stResBuf.preFits);
        eigenMatrixXToCArray(filterOutput.stAttResiduals.postFit, stResBuf.postFits);
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
    this->filterStResOutMsg.write(stResBuf, this->moduleID, currentSimNanos);
    this->filterGyroResOutMsg.write(gyroResBuf, this->moduleID, currentSimNanos);
}
