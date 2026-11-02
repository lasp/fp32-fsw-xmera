#include "flybyFilterAlgorithm.h"

#include "utilities/fsw/rigidBodyKinematics.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Core>

#include <utility>

namespace filtering::flybyFilter {

namespace {

using State = FlybyFilterAlgorithm::State;

/*! Optical-navigation heading observation: the line of sight rhat = r/|r| (the unit vector pointing from the
 *  spacecraft to the central body, inertial frame) rotated by the two heading-bias angles. Each angle is a principal
 *  rotation vector (PRV): b_across * e_along turns the line of sight by b_across [rad] toward e_across, and
 *  -b_along * e_across turns it by b_along [rad] toward e_along. The model rotates rhat by the sum of the two
 *  PRVs, addPrv(across, along): the across rotation followed by the along one. A rotation keeps the model a unit
 *  vector, like the observed heading. The terminator directions e_across and e_along are fixed for the
 *  measurement update (see terminatorFrame()), so every sigma point uses the same frame; both are zero when the
 *  bias is not modeled, which leaves the model at rhat. Satisfies the Measurement concept for use inside the
 *  SRuKF. subtract() is the plain vector difference -- the observation and model both live in R^3. */
class HeadingMeasurementModel {
   public:
    static constexpr int size = 3;

    HeadingMeasurementModel(Eigen::Vector3f observation,
                            Eigen::Matrix3f noise,
                            Eigen::Vector3f eAcross_N,
                            Eigen::Vector3f eAlong_N)
        : observed(std::move(observation)),
          measNoise(std::move(noise)),
          eAcross(std::move(eAcross_N)),
          eAlong(std::move(eAlong_N)) {}

    Eigen::Vector3f observation() const { return this->observed; }
    Eigen::Vector3f model(State const& state) const {
        Eigen::Vector3f const r = state.get<filtering::Position<3>>();
        Eigen::Vector2f const bias = state.get<filtering::Bias<2>>();
        Eigen::Vector3f const rHat = r / r.stableNorm();
        Eigen::Vector3f const prvAcross = bias(AcrossTerminator) * this->eAlong;
        Eigen::Vector3f const prvAlong = -bias(AlongTerminator) * this->eAcross;
        // prvToDcm is the frame (passive) rotation; its transpose rotates the vector.
        return prvToDcm<float>(addPrv<float>(prvAcross, prvAlong)).transpose() * rHat;
    }
    Eigen::Matrix3f noise() const { return this->measNoise; }
    static Eigen::Vector3f subtract(Eigen::Vector3f const& a, Eigen::Vector3f const& b) { return a - b; }

   private:
    Eigen::Vector3f observed;
    Eigen::Matrix3f measNoise;
    Eigen::Vector3f eAcross;  //!< [-] across-terminator unit direction, inertial frame; zero when not modeled
    Eigen::Vector3f eAlong;   //!< [-] along-terminator unit direction, inertial frame; zero when not modeled
};

static_assert(filtering::Measurement<HeadingMeasurementModel, State>);

/*! Terminator directions of a heading reading, built once per measurement update from the measured line of sight
 *  and the reading's sun direction: e_across is the sun direction's part perpendicular to the line of sight,
 *  normalized (across the terminator, toward the lit limb), and e_along = rhat x e_across (along the terminator).
 *  With no sun direction, or one within MinSunLineOfSightAngle of the line of sight (phase near 0 or 180 deg),
 *  both are zero and the heading is modeled without the bias.
 *  @return {e_across, e_along} [-] inertial frame
 *  @param rhat_N [-] measured line of sight, inertial frame
 *  @param shat_N [-] unit sun direction, inertial frame; zero when unavailable */
std::pair<Eigen::Vector3f, Eigen::Vector3f> terminatorFrame(Eigen::Vector3f const& rhat_N,
                                                            Eigen::Vector3f const& shat_N) {
    Eigen::Vector3f const rHat = rhat_N.stableNormalized();
    Eigen::Vector3f const sunAcrossLineOfSight = shat_N - (shat_N.dot(rHat) * rHat);
    float const sunAcrossNorm = sunAcrossLineOfSight.stableNorm();
    if (!(sunAcrossNorm > MinSunProjection)) {
        return {Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero()};
    }
    Eigen::Vector3f const eAcross = sunAcrossLineOfSight / sunAcrossNorm;
    return {eAcross, rHat.cross(eAcross)};
}

/*! The SRuKF Cholesky-factors the measurement noise, so it must be finite, symmetric and positive
 *  definite; a missing (zero) or corrupt covariance would otherwise poison the update.
 *  @return true iff covar_N can be used as the heading measurement noise
 *  @param covar_N [-] heading noise covariance supplied with the reading */
bool isValidHeadingCovariance(Eigen::Matrix3f const& covar_N) {
    return covar_N.allFinite() && isSymmetric<3>(covar_N) &&
           Eigen::LLT<Eigen::Matrix3f>(covar_N).info() == Eigen::Success;
}

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
    this->measurements.setDelay(config.getMeasurementDelay());
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
 *  (robust scheduling), re-seeds the heading bias if it left its bounds, and returns a post-update snapshot.
 *  @return Snapshot of post-update filter state and residuals.
 *  @param currentSeconds [s] simulation time the filter is advancing to
 *  @param headingData    [-] optical-navigation heading reading + time tag */
FlybyFilterOutput FlybyFilterAlgorithm::update(double currentSeconds, HeadingData const& headingData) {
    this->lastHeadingResiduals.valid = false;

    if (headingData.timeTag > 0 && isValidHeadingCovariance(headingData.covar_N)) {
        this->measurements.enqueue(headingData.timeTag, this->packHeadingMeasurement(headingData));
    }
    applySequentialRobust(this->measurements, *this, currentSeconds);
    this->resetBiasOutsideBounds();

    return FlybyFilterOutput{
        .filterState = this->getFilterOutput(),
        .headingResiduals = this->lastHeadingResiduals,
    };
}

/*! Propagate the state from the last-measurement anchor by dt.
 *  @return false if the propagated state/covariance is non-finite
 *  @param dt [s] elapsed time since the last measurement */
bool FlybyFilterAlgorithm::timeUpdate(float dt) { return this->srukf.timeUpdate(dt); }

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

/*! Apply a heading measurement and record its residuals. The terminator frame is built once here, from the
 *  measured line of sight and the reading's sun direction. The SRuKF returns the residuals only on a
 *  good update; an unsuccessful update yields no value and returns false to applySequential.
 *  @return true iff the update was applied (state/covariance finite)
 *  @param measurement [-] packed heading measurement (rhat_BN_N, noise) */
bool FlybyFilterAlgorithm::applyMeasurement(HeadingMeasurement const& measurement) {
    auto const [eAcross, eAlong] = terminatorFrame(measurement.rhat_BN_N, measurement.shat_N);
    HeadingMeasurementModel const model{measurement.rhat_BN_N, measurement.covar, eAcross, eAlong};

    auto const result = this->srukf.measurementUpdate(model);
    this->lastHeadingResiduals.observation = model.observation();
    if (result.has_value()) {
        this->lastHeadingResiduals.valid = measurement.valid;
        this->lastHeadingResiduals.preFit = result->preFit;
        this->lastHeadingResiduals.postFit = result->postFit;
    }
    return result.has_value();
}

/*! Pack a raw heading reading into a HeadingMeasurement whose noise covariance is the reading's own
 *  covar_N scaled by the configured measurement-noise scale. The sun direction is normalized, or zeroed when
 *  it is not finite or is zero, so the measurement model sees a unit vector or no sun direction.
 *  @return HeadingMeasurement (always valid)
 *  @param headingData [-] raw heading unit vector, noise covariance, sun direction and time tag */
HeadingMeasurement FlybyFilterAlgorithm::packHeadingMeasurement(HeadingData const& headingData) const {
    HeadingMeasurement packed;
    packed.timeTag = headingData.timeTag;
    packed.rhat_BN_N = headingData.rhat_BN_N;
    packed.covar = this->cfg.getMeasurementNoiseScale() * headingData.covar_N;
    float const sunNorm = headingData.shat_N.stableNorm();
    if (headingData.shat_N.allFinite() && sunNorm > 0.0F) {
        packed.shat_N = headingData.shat_N / sunNorm;
    }
    packed.valid = true;
    return packed;
}

/*! Re-seed the heading bias from the initial conditions when either angle left its maximum magnitude, in the
 *  working state or in the last-measurement anchor (which the next propagation starts from): both angles
 *  return to their initial values and their covariance block to the initial one, uncorrelated with the
 *  position and velocity, which keep their estimate and covariance. */
void FlybyFilterAlgorithm::resetBiasOutsideBounds() {
    float const maxAcross = this->cfg.getMaxAcrossTerminatorBias();
    float const maxAlong = this->cfg.getMaxAlongTerminatorBias();
    Eigen::Vector2f const bias = this->srukf.getState().get<filtering::Bias<2>>();
    Eigen::Vector2f const anchorBias = this->srukf.getStateAtLastMeasurement().get<filtering::Bias<2>>();

    if (!(FlybyFilterConfig::biasIsWithinBounds(bias, maxAcross, maxAlong) &&
          FlybyFilterConfig::biasIsWithinBounds(anchorBias, maxAcross, maxAlong))) {
        this->srukf.reInitializeComponent<filtering::Bias<2>>();
    }
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

Eigen::Matrix<float, FlybyFilterAlgorithm::N, FlybyFilterAlgorithm::N> FlybyFilterAlgorithm::getCovariance() const {
    return this->srukf.getCovariance();
}

}  // namespace filtering::flybyFilter
