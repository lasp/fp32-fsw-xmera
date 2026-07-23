#include "sunlineFilterAlgorithm.h"

#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/validPSDCheck.h"

#include <algorithm>
#include <utility>
#include <variant>

namespace filtering::sunlineFilter {

namespace {

using State = SunlineFilterAlgorithm::State;

/*! CSS observation = bias * H * s_hat. Satisfies the Measurement
 *  concept for use inside the SRuKF. */
class CssMeasurementModel {
   public:
    static constexpr int size = MaxCss;

    CssMeasurementModel(Eigen::Vector<double, size> observation,
                        Eigen::Matrix<double, size, 3> hMatrix,
                        Eigen::Matrix<double, size, size> noise)
        : observed(std::move(observation)), hMatrix(std::move(hMatrix)), measNoise(std::move(noise)) {}

    Eigen::Vector<double, size> observation() const { return this->observed; }
    Eigen::Vector<double, size> model(State const& state) const {
        double const bias = state.get<filtering::Bias<1>>()(0);
        Eigen::Vector3d const sHat = state.get<filtering::Position<3>>();
        return bias * (this->hMatrix * sHat);
    }
    Eigen::Matrix<double, size, size> noise() const { return this->measNoise; }
    static Eigen::Vector<double, size> subtract(Eigen::Vector<double, size> const& a,
                                                Eigen::Vector<double, size> const& b) {
        return a - b;
    }

   private:
    Eigen::Vector<double, size> observed;
    Eigen::Matrix<double, size, 3> hMatrix;
    Eigen::Matrix<double, size, size> measNoise;
};

/*! Predicted gyro observation = omega_BN_B. Satisfies the Measurement concept. */
class RateMeasurementModel {
   public:
    static constexpr int size = 3;

    RateMeasurementModel(Eigen::Vector3d observation, Eigen::Matrix3d noise)
        : observed(std::move(observation)), measNoise(std::move(noise)) {}

    Eigen::Vector3d observation() const { return this->observed; }
    static Eigen::Vector3d model(State const& state) { return state.get<filtering::Velocity<3>>(); }
    Eigen::Matrix3d noise() const { return this->measNoise; }
    static Eigen::Vector3d subtract(Eigen::Vector3d const& a, Eigen::Vector3d const& b) { return a - b; }

   private:
    Eigen::Vector3d observed;
    Eigen::Matrix3d measNoise;
};

static_assert(filtering::Measurement<CssMeasurementModel, State>);
static_assert(filtering::Measurement<RateMeasurementModel, State>);

}  // namespace

/*! Construct from a validated configuration and seed the filter runtime state.
 *  @param config [-] validated SunlineFilterConfig */
SunlineFilterAlgorithm::SunlineFilterAlgorithm(const SunlineFilterConfig& config) : cfg(config) {
    this->setConfig(config);
    this->reInitialize();
}

/*! Store the configuration and push the constant filter parameters into the SRuKF.
 *  @param config [-] validated SunlineFilterConfig */
void SunlineFilterAlgorithm::setConfig(SunlineFilterConfig const& config) {
    this->cfg = config;
    this->srukf.setAlpha(config.getAlpha());
    this->srukf.setBeta(config.getBeta());
    this->srukf.setProcessNoise(config.getProcessNoise());
    this->srukf.setInitialState(config.getInitialState());
    this->srukf.setInitialCovariance(config.getInitialCovariance());
    this->srukf.dynamics = SunlineDynamics{};
    this->srukf.reConfigure();
}

/*! Clear the internal runtime state (pending measurements and residual snapshots); the filter state
 *  and covariance are preserved.
 *  @return void */
void SunlineFilterAlgorithm::reInitializeExceptPersistentStates() {
    this->measurements.clear();
    this->lastCssResiduals = CssResidualsOutput{};
    this->lastRateResiduals = RateResidualsOutput{};
}

/*! reInitializeExceptPersistentStates() and additionally re-seed the filter state and covariance from the
 * configuration.
 *  @return void */
void SunlineFilterAlgorithm::reInitialize() {
    this->reInitializeExceptPersistentStates();
    this->srukf.reset();
}

/*! Main entrypoint. Enqueues whichever measurements are present, empties
 *  the queue through the SRuKF, then sanitizes the state.
/*! Set the fixed time step applied on every update() call.
 *  @param newDt [s] filter time step */
void SunlineFilterAlgorithm::setDt(double newDt) { this->dt = newDt; }

/*! @return the fixed time step applied on every update() call */
double SunlineFilterAlgorithm::getDt() const { return this->dt; }
 *  @return Snapshot of post-update filter state and residuals.
 *  @param currentSeconds [s] simulation time the filter is advancing to
 *  @param cssData        [-] CSS array reading + time tag
 *  @param rateData       [-] gyro reading + time tag */
SunlineFilterOutput SunlineFilterAlgorithm::update(double currentSeconds,
                                                   CssData const& cssData,
                                                   RateData const& rateData) {
    this->lastCssResiduals.valid = false;
    this->lastRateResiduals.valid = false;

    if (cssData.timeTag > 0) {
        this->measurements.enqueue(cssData.timeTag, this->packCssMeasurement(cssData));
    }
    if (rateData.timeTag > 0) {
        this->measurements.enqueue(rateData.timeTag, this->packRateMeasurement(rateData));
    }
    applySequentialRobust(this->measurements, *this, currentSeconds);
    this->srukf.setState(this->regularize(this->srukf.getState()));
    this->srukf.setStateLastMeasurement(this->regularize(this->srukf.getStateAtLastMeasurement()));

    return SunlineFilterOutput{
        .filterState = this->getFilterOutput(),
        .cssResiduals = this->lastCssResiduals,
        .rateResiduals = this->lastRateResiduals,
    };
}

/*! Propagate the state from the last-measurement anchor by dt.
 *  @return false if the propagated state/covariance is non-finite
 *  @param dt [s] elapsed time since the last measurement */
bool SunlineFilterAlgorithm::timeUpdate(double dt) { return this->srukf.timeUpdate(dt); }

/*! Fold a single measurement into the filter; dispatches per-kind.
 *  @return false if the resulting state/covariance is non-finite
 *  @param measurement [-] CSS- or rate-kind measurement to apply */
bool SunlineFilterAlgorithm::measurementUpdate(Measurement const& measurement) {
    return std::visit([this](auto const& meas) { return this->applyMeasurement(meas); }, measurement);
}

/*! Restore the filter to its last good (last-measurement) state after a bad update. */
void SunlineFilterAlgorithm::clear() {
    this->srukf.clear();
    this->lastCssResiduals.valid = false;
    this->lastRateResiduals.valid = false;
}

/*! Apply a CSS measurement and record its residuals. The SRuKF returns the residuals only on a
 *  good update; an unsuccessful update yields no value and returns false to applySequential.
 *  @return true iff the update was applied (state/covariance finite)
 *  @param measurement [-] packed CSS measurement (cosValues, H, noise) */
bool SunlineFilterAlgorithm::applyMeasurement(CssMeasurement const& measurement) {
    CssMeasurementModel const model{measurement.cssCosValues, measurement.hMatrix, measurement.covar};

    auto const result = this->srukf.measurementUpdate(model);
    this->lastCssResiduals.numberOfActiveCss = measurement.numberOfActiveCss;
    this->lastCssResiduals.observation = model.observation();
    if (result.has_value()) {
        this->lastCssResiduals.valid = measurement.valid;
        this->lastCssResiduals.preFit = result->preFit;
        this->lastCssResiduals.postFit = result->postFit;
    }
    return result.has_value();
}

/*! Apply a rate measurement and record its residuals. See applyMeasurement(CssMeasurement)
 *  for the bad-update handling.
 *  @return true iff the update was applied (state/covariance finite)
 *  @param measurement [-] packed rate measurement (omega, noise) */
bool SunlineFilterAlgorithm::applyMeasurement(RateMeasurement const& measurement) {
    RateMeasurementModel const model{measurement.omega_BN_B, measurement.covar};

    auto const result = this->srukf.measurementUpdate(model);
    this->lastRateResiduals.observation = model.observation();
    if (result.has_value()) {
        this->lastRateResiduals.valid = measurement.valid;
        this->lastRateResiduals.preFit = result->preFit;
        this->lastRateResiduals.postFit = result->postFit;
    }
    return result.has_value();
}

/*! Pack CSS readings into a CssMeasurement: active rows
 *  (cosValue > sensorUseThresh) populate the first `active` slots; H rows
 *  carry CBias * nHat vectors.
 *  @return CssMeasurement (valid = active > 0)
 *  @param cssData [-] raw CSS cos-values and time tag */
CssMeasurement SunlineFilterAlgorithm::packCssMeasurement(CssData const& cssData) const {
    double const cssMeasNoiseStd = this->cfg.getCssMeasurementNoiseStd();
    CssMeasurement packed;
    packed.timeTag = cssData.timeTag;
    packed.covar = (cssMeasNoiseStd * cssMeasNoiseStd) * Eigen::Matrix<double, MaxCss, MaxCss>::Identity();

    int active = 0;
    for (uint32_t i = 0; i < this->cfg.getNumberOfCss() && active < MaxCss; ++i) {
        if (cssData.cosValues(i) <= this->cfg.getSensorThreshold()) {
            continue;
        }
        packed.cssCosValues(active) = cssData.cosValues(i);
        packed.hMatrix.row(active) = this->cfg.getCssScaleFactor()(i) * this->cfg.getCssNHat().row(i);
        active += 1;
    }
    packed.numberOfActiveCss = active;
    packed.valid = active > 0;
    return packed;
}

/*! Pack a raw gyro reading into a RateMeasurement with diagonal noise covar.
 *  @return RateMeasurement (always valid)
 *  @param rateData [-] raw rate vector and time tag */
RateMeasurement SunlineFilterAlgorithm::packRateMeasurement(RateData const& rateData) const {
    RateMeasurement packed;
    packed.timeTag = rateData.timeTag;
    packed.omega_BN_B = rateData.rate;
    double const gyroMeasNoiseStd = this->cfg.getGyroMeasurementNoiseStd();
    packed.covar = (gyroMeasNoiseStd * gyroMeasNoiseStd) * Eigen::Matrix3d::Identity();
    packed.valid = true;
    return packed;
}

/*! Renormalize the heading and clamp the bias to its configured bounds.
 *  @return sanitized copy of State
 *  @param state [-] state to regularize */
SunlineFilterAlgorithm::State SunlineFilterAlgorithm::regularize(State const& state) const {
    State outputState = state;
    outputState.set<filtering::Position<3>>(outputState.get<filtering::Position<3>>().normalized());
    Eigen::Vector<double, 1> biasVec;
    biasVec(0) = std::clamp(
        outputState.get<filtering::Bias<1>>()(0), this->cfg.getBiasLowerBound(), this->cfg.getBiasUpperBound());
    outputState.set<filtering::Bias<1>>(biasVec);
    return outputState;
}

/*! Bundle the SRuKF's current state and covariance into the output POD.
 *  @return FilterStateOutput */
FilterStateOutput SunlineFilterAlgorithm::getFilterOutput() const {
    FilterStateOutput filterOutput;
    filterOutput.state = this->srukf.getState().raw();
    filterOutput.covariance = this->srukf.getCovariance();
    return filterOutput;
}

/*! @return const reference to the latest CSS residuals snapshot */
CssResidualsOutput const& SunlineFilterAlgorithm::getLastCssResiduals() const { return this->lastCssResiduals; }
/*! @return const reference to the latest rate residuals snapshot */
RateResidualsOutput const& SunlineFilterAlgorithm::getLastRateResiduals() const { return this->lastRateResiduals; }

/*! @return current filter state (post-sanitization) */
SunlineFilterAlgorithm::State SunlineFilterAlgorithm::getState() const { return this->srukf.getState(); }
/*! @return current full covariance */
Eigen::Matrix<double, SunlineFilterAlgorithm::N, SunlineFilterAlgorithm::N> SunlineFilterAlgorithm::getCovariance()
    const {
    return this->srukf.getCovariance();
}

}  // namespace filtering::sunlineFilter
