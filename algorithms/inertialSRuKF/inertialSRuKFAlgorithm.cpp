#include "inertialSRuKFAlgorithm.h"

#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/rigidBodyKinematics.hpp"
#include "utilities/fsw/validPSDCheck.h"

#include <variant>

namespace filtering::inertialSRuKF {

namespace {

using State = InertialSRuKFAlgorithm::State;

/*! Star-tracker attitude observation = sigma_BN. Satisfies the Measurement
 *  concept for use inside the SRuKF. subtract() forms the MRP-difference
 *  (shadow-set aware) so the innovation stays small across the |sigma| = 1
 *  boundary. */
struct StAttMeasurementModel {
    static constexpr int size = 3;

    // Concept-model aggregate: these fields are populated directly by
    // applyMeasurement() and read back through the accessors below to satisfy
    // the filtering::Measurement concept. Public data is the point of the type,
    // so the private-member guidance does not apply here.
    // NOLINTBEGIN(misc-non-private-member-variables-in-classes)
    Eigen::Vector3d observed = Eigen::Vector3d::Zero();
    Eigen::Matrix3d measNoise = Eigen::Matrix3d::Identity();
    // NOLINTEND(misc-non-private-member-variables-in-classes)

    Eigen::Vector3d observation() const { return this->observed; }
    static Eigen::Vector3d model(State const& state) { return state.get<filtering::MrpAttitude<3>>(); }
    Eigen::Matrix3d noise() const { return this->measNoise; }
    static Eigen::Vector3d subtract(Eigen::Vector3d const& a, Eigen::Vector3d const& b) { return subMrp(a, b); }
};

/*! Predicted gyro observation = omega_BN_B. Satisfies the Measurement concept. */
struct RateMeasurementModel {
    static constexpr int size = 3;

    // Concept-model aggregate; see StAttMeasurementModel for the rationale.
    // NOLINTBEGIN(misc-non-private-member-variables-in-classes)
    Eigen::Vector3d observed = Eigen::Vector3d::Zero();
    Eigen::Matrix3d measNoise = Eigen::Matrix3d::Identity();
    // NOLINTEND(misc-non-private-member-variables-in-classes)

    Eigen::Vector3d observation() const { return this->observed; }
    static Eigen::Vector3d model(State const& state) { return state.get<filtering::AngularRate<3>>(); }
    Eigen::Matrix3d noise() const { return this->measNoise; }
    static Eigen::Vector3d subtract(Eigen::Vector3d const& a, Eigen::Vector3d const& b) { return a - b; }
};

static_assert(filtering::Measurement<StAttMeasurementModel, State>);
static_assert(filtering::Measurement<RateMeasurementModel, State>);

}  // namespace

/*! Construct from a validated configuration and seed the filter runtime state.
 *  @param config [-] validated InertialSRuKFConfig */
InertialSRuKFAlgorithm::InertialSRuKFAlgorithm(const InertialSRuKFConfig& config) : cfg(config) {
    this->setConfig(config);
    this->reInitializeAll();
}

/*! Store the configuration and push the constant filter parameters into the SRuKF.
 *  @param config [-] validated InertialSRuKFConfig */
void InertialSRuKFAlgorithm::setConfig(InertialSRuKFConfig const& config) {
    this->cfg = config;
    this->srukf.setAlpha(config.getAlpha());
    this->srukf.setBeta(config.getBeta());
    this->srukf.setProcessNoise(config.getProcessNoise());
    this->srukf.setInitialState(config.getInitialState());
    this->srukf.setInitialCovariance(config.getInitialCovariance());
    this->srukf.dynamics = InertialDynamics{};
    this->srukf.reConfigure();
}

/*! Clear the internal runtime state (pending measurements and residual snapshots); the filter state
 *  and covariance are preserved.
 *  @return void */
void InertialSRuKFAlgorithm::reInitialize() {
    this->measurements.clear();
    this->lastStAttResiduals = StAttResidualsOutput{};
    this->lastRateResiduals = RateResidualsOutput{};
}

/*! reInitialize() and additionally re-seed the filter state and covariance from the configuration.
 *  @return void */
void InertialSRuKFAlgorithm::reInitializeAll() {
    this->reInitialize();
    this->srukf.reset();
}

/*! Main entrypoint. Enqueues whichever measurements are present, empties
 *  the queue through the SRuKF, then sanitizes the state.
 *  @return Snapshot of post-update filter state and residuals.
 *  @param currentSeconds [s] simulation time the filter is advancing to
 *  @param stAttData      [-] star-tracker attitude reading + time tag
 *  @param rateData       [-] gyro reading + time tag */
InertialSRuKFOutput InertialSRuKFAlgorithm::update(double currentSeconds,
                                                   StAttData const& stAttData,
                                                   RateData const& rateData) {
    this->lastStAttResiduals.valid = false;
    this->lastRateResiduals.valid = false;

    if (stAttData.timeTag > 0) {
        this->measurements.enqueue(stAttData.timeTag, this->packStAttMeasurement(stAttData));
    }
    if (rateData.timeTag > 0) {
        this->measurements.enqueue(rateData.timeTag, this->packRateMeasurement(rateData));
    }
    applySequentialRobust(this->measurements, *this, currentSeconds);
    this->srukf.setState(this->regularize(this->srukf.getState()));
    this->srukf.setStateLastMeasurement(this->regularize(this->srukf.getStateAtLastMeasurement()));

    return InertialSRuKFOutput{
        .filterState = this->getFilterOutput(),
        .stAttResiduals = this->lastStAttResiduals,
        .rateResiduals = this->lastRateResiduals,
    };
}

/*! Propagate the state from the last-measurement anchor by dt.
 *  @return false if the propagated state/covariance is non-finite
 *  @param dt [s] elapsed time since the last measurement */
bool InertialSRuKFAlgorithm::timeUpdate(double dt) { return this->srukf.timeUpdate(dt); }

/*! Fold a single measurement into the filter; dispatches per-kind.
 *  @return false if the resulting state/covariance is non-finite
 *  @param measurement [-] ST- or rate-kind measurement to apply */
bool InertialSRuKFAlgorithm::measurementUpdate(Measurement const& measurement) {
    return std::visit([this](auto const& meas) { return this->applyMeasurement(meas); }, measurement);
}

/*! Restore the filter to its last good (last-measurement) state after a bad update. */
void InertialSRuKFAlgorithm::clear() {
    this->srukf.clear();
    this->lastStAttResiduals.valid = false;
    this->lastRateResiduals.valid = false;
}

/*! Apply a star-tracker attitude measurement and record its residuals. The SRuKF returns
 *  the residuals only on a good update; an unsuccessful update yields no value and returns
 *  false to applySequential.
 *  @return true iff the update was applied (state/covariance finite)
 *  @param measurement [-] packed ST measurement (sigma_BN, noise) */
bool InertialSRuKFAlgorithm::applyMeasurement(StAttMeasurement const& measurement) {
    StAttMeasurementModel model;
    model.observed = measurement.sigma_BN;
    model.measNoise = measurement.covar;

    auto const result = this->srukf.measurementUpdate(model);
    this->lastStAttResiduals.observation = model.observed;
    if (result.has_value()) {
        this->lastStAttResiduals.valid = measurement.valid;
        this->lastStAttResiduals.preFit = result->preFit;
        this->lastStAttResiduals.postFit = result->postFit;
    }
    return result.has_value();
}

/*! Apply a rate measurement and record its residuals. See applyMeasurement(StAttMeasurement)
 *  for the bad-update handling.
 *  @return true iff the update was applied (state/covariance finite)
 *  @param measurement [-] packed rate measurement (omega, noise) */
bool InertialSRuKFAlgorithm::applyMeasurement(RateMeasurement const& measurement) {
    RateMeasurementModel model;
    model.observed = measurement.omega_BN_B;
    model.measNoise = measurement.covar;

    auto const result = this->srukf.measurementUpdate(model);
    this->lastRateResiduals.observation = model.observed;
    if (result.has_value()) {
        this->lastRateResiduals.valid = measurement.valid;
        this->lastRateResiduals.preFit = result->preFit;
        this->lastRateResiduals.postFit = result->postFit;
    }
    return result.has_value();
}

/*! Pack a raw star-tracker attitude reading into an StAttMeasurement with diagonal noise covar.
 *  @return StAttMeasurement (always valid)
 *  @param stAttData [-] raw MRP attitude and time tag */
StAttMeasurement InertialSRuKFAlgorithm::packStAttMeasurement(StAttData const& stAttData) const {
    StAttMeasurement packed;
    packed.timeTag = stAttData.timeTag;
    packed.sigma_BN = stAttData.sigma_BN;
    double const stMeasNoiseStd = this->cfg.getStMeasurementNoiseStd();
    packed.covar = (stMeasNoiseStd * stMeasNoiseStd) * Eigen::Matrix3d::Identity();
    packed.valid = true;
    return packed;
}

/*! Pack a raw gyro reading into a RateMeasurement with diagonal noise covar.
 *  @return RateMeasurement (always valid)
 *  @param rateData [-] raw rate vector and time tag */
RateMeasurement InertialSRuKFAlgorithm::packRateMeasurement(RateData const& rateData) const {
    RateMeasurement packed;
    packed.timeTag = rateData.timeTag;
    packed.omega_BN_B = rateData.rate;
    double const gyroMeasNoiseStd = this->cfg.getGyroMeasurementNoiseStd();
    packed.covar = (gyroMeasNoiseStd * gyroMeasNoiseStd) * Eigen::Matrix3d::Identity();
    packed.valid = true;
    return packed;
}

/*! Map the MRP attitude to its inner shadow set when |sigma| > 1; the body rate is left untouched.
 *  @return sanitized copy of State
 *  @param state [-] state to regularize */
InertialSRuKFAlgorithm::State InertialSRuKFAlgorithm::regularize(State const& state) const {
    State outputState = state;
    outputState.set<filtering::MrpAttitude<3>>(mrpSwitch(outputState.get<filtering::MrpAttitude<3>>(), 1.0));
    return outputState;
}

/*! Bundle the SRuKF's current state and covariance into the output POD.
 *  @return FilterStateOutput */
FilterStateOutput InertialSRuKFAlgorithm::getFilterOutput() const {
    FilterStateOutput filterOutput;
    filterOutput.state = this->srukf.getState().raw();
    filterOutput.covariance = this->srukf.getCovariance();
    return filterOutput;
}

/*! @return const reference to the latest ST residuals snapshot */
StAttResidualsOutput const& InertialSRuKFAlgorithm::getLastStAttResiduals() const { return this->lastStAttResiduals; }
/*! @return const reference to the latest rate residuals snapshot */
RateResidualsOutput const& InertialSRuKFAlgorithm::getLastRateResiduals() const { return this->lastRateResiduals; }

/*! @return current filter state (post-sanitization) */
InertialSRuKFAlgorithm::State InertialSRuKFAlgorithm::getState() const { return this->srukf.getState(); }
/*! @return current full covariance */
Eigen::Matrix<double, InertialSRuKFAlgorithm::N, InertialSRuKFAlgorithm::N> InertialSRuKFAlgorithm::getCovariance()
    const {
    return this->srukf.getCovariance();
}

}  // namespace filtering::inertialSRuKF
