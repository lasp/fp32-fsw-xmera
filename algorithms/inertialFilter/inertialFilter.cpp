#include "inertialFilter.h"

#include "inertialFilterAlgorithm.h"
#include "utilities/fsw/timeConstants.h"
#include "utilities/xmera/xmeraLifecycleException.h"

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
    this->processNoise = Eigen::MatrixXf::Zero(n, n);
    this->initialState = Eigen::VectorXf::Zero(n);
    this->initialCovariance = Eigen::MatrixXf::Identity(n, n);
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
                                                     InertialState{Eigen::Vector<float, n>(this->initialState)},
                                                     StateMatrix(this->initialCovariance),
                                                     this->stMeasurementNoiseStd,
                                                     this->rateMeasurementNoiseStd);
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

/*! Read the star-tracker message, call algorithm update, and write the output state and
 *  residuals.
 *  @return void
 *  @param currentSimNanos [ns] sim time the filter is advancing to */
void InertialFilter::updateState(uint64_t currentSimNanos) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("InertialFilter reset() has not been called.");
    }

    double const currentSeconds = static_cast<double>(currentSimNanos) * kNano2Sec;

    StAttData stAttData{};
    RateData rateData{};

    if (auto const stPayload = this->stAttInMsg(); static_cast<double>(stPayload.timeTag) > this->lastStTimeTag) {
        stAttData.timeTag = static_cast<double>(stPayload.timeTag);
        stAttData.sigma_BN = cArrayToEigenVector(stPayload.MRP_BdyInrtl);
        rateData.timeTag = stAttData.timeTag;
        rateData.rate = cArrayToEigenVector(stPayload.omega_BN_B);
        this->lastStTimeTag = stAttData.timeTag;
    }

    InertialFilterOutput const filterOutput = this->algorithm->update(currentSeconds, stAttData, rateData);
    this->writeOutputMessages(currentSimNanos, filterOutput);
}

/*! Write the algorithm's output to the four xmera output messages.
 *  @return void
 *  @param currentSimNanos [ns] sim time provided to the outgoing messages
 *  @param filterOutput    [-]  filter data returned by algorithm */
void InertialFilter::writeOutputMessages(uint64_t currentSimNanos, InertialFilterOutput const& filterOutput) {
    NavAttMsgF32Payload navAttBuf{};
    FilterMsgF32Payload filterBuf{};
    FilterResidualsMsgF32Payload stResBuf{};
    FilterResidualsMsgF32Payload rateResBuf{};

    double const timeTag = static_cast<double>(currentSimNanos) * kNano2Sec;

    navAttBuf.timeTag = timeTag;
    eigenVectorToCArray(filterOutput.filterState.state.head<3>().eval(), navAttBuf.sigma_BN);
    eigenVectorToCArray(filterOutput.filterState.state.segment<3>(3).eval(), navAttBuf.omega_BN_B);

    filterBuf.timeTag = timeTag;
    filterBuf.numberOfStates = InertialFilterAlgorithm::N;
    eigenMatrixXToCArray(filterOutput.filterState.state.cast<double>().eval(), filterBuf.state);
    eigenMatrixXToCArray(filterOutput.filterState.covariance.cast<double>().eval(), filterBuf.covar);

    if (filterOutput.stAttResiduals.valid) {
        stResBuf.timeTag = timeTag;
        stResBuf.valid = true;
        stResBuf.numberOfObservations = 1;
        stResBuf.sizeOfObservations = 3;
        eigenMatrixXToCArray(filterOutput.stAttResiduals.observation.cast<double>().eval(), stResBuf.observation);
        eigenMatrixXToCArray(filterOutput.stAttResiduals.preFit.cast<double>().eval(), stResBuf.preFits);
        eigenMatrixXToCArray(filterOutput.stAttResiduals.postFit.cast<double>().eval(), stResBuf.postFits);
    }
    if (filterOutput.rateResiduals.valid) {
        rateResBuf.timeTag = timeTag;
        rateResBuf.valid = true;
        rateResBuf.numberOfObservations = 1;
        rateResBuf.sizeOfObservations = 3;
        eigenMatrixXToCArray(filterOutput.rateResiduals.observation.cast<double>().eval(), rateResBuf.observation);
        eigenMatrixXToCArray(filterOutput.rateResiduals.preFit.cast<double>().eval(), rateResBuf.preFits);
        eigenMatrixXToCArray(filterOutput.rateResiduals.postFit.cast<double>().eval(), rateResBuf.postFits);
    }

    this->navAttOutMsg.write(navAttBuf, this->moduleID, currentSimNanos);
    this->filterOutMsg.write(filterBuf, this->moduleID, currentSimNanos);
    this->filterStResOutMsg.write(stResBuf, this->moduleID, currentSimNanos);
    this->filterRateResOutMsg.write(rateResBuf, this->moduleID, currentSimNanos);
}
