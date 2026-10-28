#include "flybyFilter.h"

#include "flybyFilterAlgorithm.h"
#include "utilities/xmera/xmeraLifecycleException.h"

#include <architecture/utilities/eigenSupport.h>
#include <utilities/fsw/freestandingIsFinite.hpp>
#include <utilities/fsw/rigidBodyKinematics.hpp>
#include <utilities/fsw/timeConversion.hpp>

#include <Eigen/Core>

#include <stdexcept>

using filtering::flybyFilter::FlybyFilterAlgorithm;
using filtering::flybyFilter::FlybyFilterConfig;
using filtering::flybyFilter::FlybyFilterOutput;
using filtering::flybyFilter::FlybyState;
using filtering::flybyFilter::HeadingData;
using filtering::flybyFilter::StateMatrix;
using Position = filtering::Position<3>;
using Velocity = filtering::Velocity<3>;

FlybyFilter::FlybyFilter()
    : processNoise(StateMatrix::Zero()),
      initialState(StateColumn::Zero()),
      initialCovariance(StateMatrix::Identity()) {}

FlybyFilter::~FlybyFilter() = default;

/*! Validate message connections, convert the SI public properties into internal (km) units, build a
 *  validated config, and construct the algorithm (its constructor seeds the filter state/covariance).
 *  The seeded estimate is published at once, so a consumer that runs before the first update() reads
 *  the initial state instead of an unwritten (zero) message.
 *  @return void
 *  @param currentSimNanos [ns] sim time at which reset was called */
void FlybyFilter::reset(uint64_t currentSimNanos) {
    if (!this->opNavHeadingMsg.isLinked()) {
        throw std::invalid_argument("flybyFilter.opNavHeadingMsg wasn't connected.");
    }
    // Guarded here rather than in the Config: unitConversion never reaches the algorithm, but
    // writeOutputMessages() divides by it, so a zero or negative scale would silently publish
    // infinities instead of failing at reset().
    if (!fsw::is_finite(this->unitConversion) || this->unitConversion <= 0.0F) {
        throw std::invalid_argument("flybyFilter.unitConversion must be positive.");
    }

    float const uc = this->unitConversion;
    float const uc3 = uc * uc * uc;

    // SI -> internal: the position and velocity x uc (their covariance and process noise rows and columns
    // x uc), mu x uc^3. The heading-bias angles are not scaled.
    StateColumn internalStateVec{this->initialState};
    StateMatrix internalCovariance{this->initialCovariance};
    StateMatrix internalProcessNoise{this->processNoise};
    FlybyState::scaleComponent<Position>(internalStateVec, uc);
    FlybyState::scaleComponent<Velocity>(internalStateVec, uc);
    FlybyState::scaleCovarianceComponent<Position>(internalCovariance, uc);
    FlybyState::scaleCovarianceComponent<Velocity>(internalCovariance, uc);
    FlybyState::scaleCovarianceComponent<Position>(internalProcessNoise, uc);
    FlybyState::scaleCovarianceComponent<Velocity>(internalProcessNoise, uc);
    FlybyState const internalState{internalStateVec};

    auto const config = FlybyFilterConfig::create(this->alpha,
                                                  this->beta,
                                                  this->mu * uc3,
                                                  internalProcessNoise,
                                                  internalState,
                                                  internalCovariance,
                                                  this->measurementNoiseScale,
                                                  this->maxAcrossTerminatorBias,
                                                  this->maxAlongTerminatorBias,
                                                  this->measurementDelay);
    this->algorithm = std::make_unique<FlybyFilterAlgorithm>(config);
    this->lastHeadingTimeTag = 0;

    this->writeOutputMessages(
        currentSimNanos, FlybyFilterOutput{.filterState = this->algorithm->getFilterOutput(), .headingResiduals = {}});
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

/*! Read the heading message (with the sun direction from the attitude message, when it is connected and
 *  written), call the algorithm update, and write the output state and residuals.
 *  @return void
 *  @param currentSimNanos [ns] sim time the filter is advancing to */
void FlybyFilter::updateState(uint64_t currentSimNanos) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("FlybyFilter reset() has not been called.");
    }

    double const currentSeconds = fsw::nanosToSeconds(currentSimNanos);

    HeadingData headingData{};
    if (auto const payload = this->opNavHeadingMsg(); payload.valid && payload.timeTag > this->lastHeadingTimeTag) {
        headingData.timeTag = payload.timeTag;
        headingData.rhat_BN_N = cArrayToEigenVector(payload.rhat_BN_N).cast<float>();
        headingData.covar_N = cArrayToEigenMatrix3(payload.covar_N);
        if (this->navAttInMsg.isLinked() && this->navAttInMsg.isWritten()) {
            NavAttMsgF32Payload const navAtt = this->navAttInMsg();
            Eigen::Matrix3f const dcm_BN = mrpToDcm(cArrayToEigenVector(navAtt.sigma_BN).cast<float>().eval());
            headingData.shat_N = dcm_BN.transpose() * cArrayToEigenVector(navAtt.vehSunPntBdy).cast<float>();
        }
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
    NavTransMsgF32Payload navTransBuf{};
    FilterMsgF32Payload filterBuf{};
    FilterResidualsMsgF32Payload resBuf{};

    double const timeTag = fsw::nanosToSeconds(currentSimNanos);
    double const invUc = 1.0 / static_cast<double>(this->unitConversion);
    Eigen::Matrix<double, FlybyFilterAlgorithm::N, 1> stateSI = filterOutput.filterState.state.cast<double>();
    Eigen::Matrix<double, FlybyFilterAlgorithm::N, FlybyFilterAlgorithm::N> covarSI =
        filterOutput.filterState.covariance.cast<double>();
    FlybyState::scaleComponent<Position>(stateSI, invUc);
    FlybyState::scaleComponent<Velocity>(stateSI, invUc);
    FlybyState::scaleCovarianceComponent<Position>(covarSI, invUc);
    FlybyState::scaleCovarianceComponent<Velocity>(covarSI, invUc);

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
        eigenMatrixXToCArray(filterOutput.headingResiduals.observation.cast<double>().eval(), resBuf.observation);
        eigenMatrixXToCArray(filterOutput.headingResiduals.preFit.cast<double>().eval(), resBuf.preFits);
        eigenMatrixXToCArray(filterOutput.headingResiduals.postFit.cast<double>().eval(), resBuf.postFits);
    }

    this->navTransOutMsg.write(navTransBuf, this->moduleID, currentSimNanos);
    this->filterOutMsg.write(filterBuf, this->moduleID, currentSimNanos);
    this->filterResOutMsg.write(resBuf, this->moduleID, currentSimNanos);
}
