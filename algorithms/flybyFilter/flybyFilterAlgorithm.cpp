#include "flybyFilterAlgorithm.h"

#include <Eigen/Core>

#include <utility>

namespace filtering::flybyFilter {

namespace {

using State = FlybyFilterAlgorithm::State;

/*! Optical-navigation heading observation = r/|r| (the unit vector pointing from the spacecraft to
 *  the central body, inertial frame). Satisfies the Measurement concept for use inside the SRuKF.
 *  subtract() is the plain vector difference -- the observation and model both live in R^3. */
class HeadingMeasurementModel {
   public:
    static constexpr int size = 3;

    HeadingMeasurementModel(Eigen::Vector3d observation, Eigen::Matrix3d noise)
        : observed(std::move(observation)), measNoise(std::move(noise)) {}

    Eigen::Vector3d observation() const { return this->observed; }
    static Eigen::Vector3d model(State const& state) {
        Eigen::Vector3d const r = state.get<filtering::Position<3>>();
        return r / r.norm();
    }
    Eigen::Matrix3d noise() const { return this->measNoise; }
    static Eigen::Vector3d subtract(Eigen::Vector3d const& a, Eigen::Vector3d const& b) { return a - b; }

   private:
    Eigen::Vector3d observed;
    Eigen::Matrix3d measNoise;
};

static_assert(filtering::Measurement<HeadingMeasurementModel, State>);

}  // namespace

/*! Construct from a validated configuration and seed the filter runtime state.
 *  @param config [-] validated FlybyFilterConfig */
FlybyFilterAlgorithm::FlybyFilterAlgorithm(const FlybyFilterConfig& config) : cfg(config) {
    this->setConfig(config);
    this->reInitialize();
}

/*! Store the configuration and push the constant filter parameters (including the mu-carrying
 *  dynamics functor) into the SRuKF.
 *  @param config [-] validated FlybyFilterConfig */
void FlybyFilterAlgorithm::setConfig(FlybyFilterConfig const& config) {
    this->cfg = config;
    this->srukf.setAlpha(config.getAlpha());
    this->srukf.setBeta(config.getBeta());
    this->srukf.setProcessNoise(config.getProcessNoise());
    this->srukf.setInitialState(config.getInitialState());
    this->srukf.setInitialCovariance(config.getInitialCovariance());
    this->srukf.dynamics = FlybyDynamics{config.getMu()};
    this->srukf.configure();
}

/*! Clear the internal runtime state (pending measurements and residual snapshot); the filter state
 *  and covariance are preserved.
 *  @return void */
void FlybyFilterAlgorithm::reInitializeExceptPersistentStates() {
    this->measurements.clear();
    this->lastHeadingResiduals = HeadingResidualsOutput{};
}

/*! reInitializeExceptPersistentStates() and additionally re-seed the filter state and covariance from the
 * configuration.
 *  @return void */
void FlybyFilterAlgorithm::reInitialize() {
    this->reInitializeExceptPersistentStates();
    this->srukf.reInitialize();
}

/*! Main entrypoint. Enqueues a fresh heading reading if present, empties the queue through the SRuKF
 *  (robust scheduling), and returns a post-update snapshot.
 *  @return Snapshot of post-update filter state and residuals.
 *  @param currentSeconds [s] simulation time the filter is advancing to
 *  @param headingData    [-] optical-navigation heading reading + time tag */
FlybyFilterOutput FlybyFilterAlgorithm::update(double currentSeconds, HeadingData const& headingData) {
    this->lastHeadingResiduals.valid = false;

    if (headingData.timeTag > 0) {
        this->measurements.enqueue(headingData.timeTag, this->packHeadingMeasurement(headingData));
    }
    applySequentialRobust(this->measurements, *this, currentSeconds);

    return FlybyFilterOutput{
        .filterState = this->getFilterOutput(),
        .headingResiduals = this->lastHeadingResiduals,
    };
}

/*! Propagate the state from the last-measurement anchor by dt.
 *  @return false if the propagated state/covariance is non-finite
 *  @param dt [s] elapsed time since the last measurement */
bool FlybyFilterAlgorithm::timeUpdate(double dt) { return this->srukf.timeUpdate(dt); }

/*! Fold a single heading measurement into the filter.
 *  @return false if the resulting state/covariance is non-finite
 *  @param measurement [-] heading measurement to apply */
bool FlybyFilterAlgorithm::measurementUpdate(Measurement const& measurement) {
    return this->applyMeasurement(measurement);
}

/*! Restore the filter to its last good (last-measurement) state after a bad update. */
void FlybyFilterAlgorithm::clear() {
    this->srukf.clear();
    this->lastHeadingResiduals.valid = false;
}

/*! Apply a heading measurement and record its residuals. The SRuKF returns the residuals only on a
 *  good update; an unsuccessful update yields no value and returns false to applySequential.
 *  @return true iff the update was applied (state/covariance finite)
 *  @param measurement [-] packed heading measurement (rhat_BN_N, noise) */
bool FlybyFilterAlgorithm::applyMeasurement(HeadingMeasurement const& measurement) {
    HeadingMeasurementModel const model{measurement.rhat_BN_N, measurement.covar};

    auto const result = this->srukf.measurementUpdate(model);
    this->lastHeadingResiduals.observation = model.observation();
    if (result.has_value()) {
        this->lastHeadingResiduals.valid = measurement.valid;
        this->lastHeadingResiduals.preFit = result->preFit;
        this->lastHeadingResiduals.postFit = result->postFit;
    }
    return result.has_value();
}

/*! Pack a raw heading reading into a HeadingMeasurement with diagonal noise covariance built from
 *  the configured measurement-noise std.
 *  @return HeadingMeasurement (always valid)
 *  @param headingData [-] raw heading unit vector and time tag */
HeadingMeasurement FlybyFilterAlgorithm::packHeadingMeasurement(HeadingData const& headingData) const {
    HeadingMeasurement packed;
    packed.timeTag = headingData.timeTag;
    packed.rhat_BN_N = headingData.rhat_BN_N;
    double const std = this->cfg.getHeadingMeasurementNoiseStd();
    packed.covar = (std * std) * Eigen::Matrix3d::Identity();
    packed.valid = true;
    return packed;
}

/*! @return current filter state and covariance snapshot */
FilterStateOutput FlybyFilterAlgorithm::getFilterOutput() const {
    FilterStateOutput out;
    out.state = this->srukf.getState().raw();
    out.covariance = this->srukf.getCovariance();
    return out;
}

HeadingResidualsOutput const& FlybyFilterAlgorithm::getLastHeadingResiduals() const {
    return this->lastHeadingResiduals;
}

FlybyFilterAlgorithm::State FlybyFilterAlgorithm::getState() const { return this->srukf.getState(); }

Eigen::Matrix<double, FlybyFilterAlgorithm::N, FlybyFilterAlgorithm::N> FlybyFilterAlgorithm::getCovariance() const {
    return this->srukf.getCovariance();
}

}  // namespace filtering::flybyFilter
