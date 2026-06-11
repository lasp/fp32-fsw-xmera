#include "sunlineSRuKFAlgorithm.h"

#include "utilities/freestandingInvalidArgument.h"
#include "utilities/validPSDCheck.h"

#include <algorithm>
#include <variant>

namespace filtering::sunlineSRuKF {

namespace {

using State = SunlineSRuKFAlgorithm::State;

/*! CSS observation = bias * H * s_hat. Satisfies the Measurement
 *  concept for use inside the SRuKF. */
struct CssMeasurementModel {
    static constexpr int size = MaxCss;

    // Concept-model aggregate: these fields are populated directly by
    // applyMeasurement() and read back through the accessors below to satisfy
    // the filtering::Measurement concept. Public data is the point of the type,
    // so the private-member guidance does not apply here.
    // NOLINTBEGIN(misc-non-private-member-variables-in-classes)
    Eigen::Vector<double, MaxCss> observed = Eigen::Vector<double, MaxCss>::Zero();
    Eigen::Matrix<double, MaxCss, 3> hMatrix = Eigen::Matrix<double, MaxCss, 3>::Zero();
    Eigen::Matrix<double, MaxCss, MaxCss> measNoise = Eigen::Matrix<double, MaxCss, MaxCss>::Identity();
    // NOLINTEND(misc-non-private-member-variables-in-classes)

    Eigen::Vector<double, MaxCss> observation() const { return this->observed; }
    Eigen::Vector<double, MaxCss> model(State const& state) const {
        double const bias = state.get<filtering::Bias<1>>()(0);
        Eigen::Vector3d const sHat = state.get<filtering::Position<3>>();
        return bias * (this->hMatrix * sHat);
    }
    Eigen::Matrix<double, MaxCss, MaxCss> noise() const { return this->measNoise; }
    static Eigen::Vector<double, MaxCss> subtract(Eigen::Vector<double, MaxCss> const& a,
                                                  Eigen::Vector<double, MaxCss> const& b) {
        return a - b;
    }
};

/*! Predicted gyro observation = omega_BN_B. Satisfies the Measurement concept. */
struct RateMeasurementModel {
    static constexpr int size = 3;

    // Concept-model aggregate; see CssMeasurementModel for the rationale.
    // NOLINTBEGIN(misc-non-private-member-variables-in-classes)
    Eigen::Vector3d observed = Eigen::Vector3d::Zero();
    Eigen::Matrix3d measNoise = Eigen::Matrix3d::Identity();
    // NOLINTEND(misc-non-private-member-variables-in-classes)

    Eigen::Vector3d observation() const { return this->observed; }
    static Eigen::Vector3d model(State const& state) { return state.get<filtering::Velocity<3>>(); }
    Eigen::Matrix3d noise() const { return this->measNoise; }
    static Eigen::Vector3d subtract(Eigen::Vector3d const& a, Eigen::Vector3d const& b) { return a - b; }
};

static_assert(filtering::Measurement<CssMeasurementModel, State>);
static_assert(filtering::Measurement<RateMeasurementModel, State>);

}  // namespace

/*! Construct from a validated configuration and seed the filter runtime state.
 *  @param config [-] validated SunlineSRuKFConfig */
SunlineSRuKFAlgorithm::SunlineSRuKFAlgorithm(const SunlineSRuKFConfig& config) : cfg(config) {
    this->setConfig(config);
    this->reInitialize();
}

/*! Swap in a new validated configuration. Takes effect at the next reInitialize().
 *  @param config [-] validated SunlineSRuKFConfig */
void SunlineSRuKFAlgorithm::setConfig(SunlineSRuKFConfig const& config) { this->cfg = config; }

/*! Re-seed the filter from the configuration and clear any state carried from a previous run.
 *  @return void */
void SunlineSRuKFAlgorithm::reInitialize() {
    this->srukf.setAlpha(this->cfg.getAlpha());
    this->srukf.setBeta(this->cfg.getBeta());
    this->srukf.setProcessNoise(this->cfg.getProcessNoise());
    this->srukf.setInitialState(this->cfg.getInitialState());
    this->srukf.setInitialCovariance(this->cfg.getInitialCovariance());
    this->srukf.dynamics = SunlineDynamics{};

    this->srukf.reset();
    this->measurements.clear();
    this->lastCssResiduals = CssResidualsOutput{};
    this->lastRateResiduals = RateResidualsOutput{};
}

/*! Main entrypoint. Enqueues whichever measurements are present, empties
 *  the queue through the SRuKF, then sanitizes the state.
 *  @return Snapshot of post-update filter state and residuals.
 *  @param currentSeconds [s] simulation time the filter is advancing to
 *  @param cssData        [-] CSS array reading + validity flag
 *  @param rateData       [-] gyro reading + validity flag */
SunlineSRuKFOutput SunlineSRuKFAlgorithm::update(double currentSeconds,
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
    applySequential(this->measurements, *this, currentSeconds);
    this->srukf.setState(this->regularize(this->srukf.getState()));
    this->srukf.setStateLastMeasurement(this->regularize(this->srukf.getStateAtLastMeasurement()));

    return SunlineSRuKFOutput{
        .filterState = this->getFilterOutput(),
        .cssResiduals = this->lastCssResiduals,
        .rateResiduals = this->lastRateResiduals,
    };
}

/*! Propagate the state from the last-measurement anchor by dt.
 *  @return void
 *  @param dt [s] elapsed time since the last measurement */
void SunlineSRuKFAlgorithm::timeUpdate(double dt) { this->srukf.timeUpdate(dt); }

/*! Fold a single measurement into the filter; dispatches per-kind.
 *  @return void
 *  @param measurement [-] CSS- or rate-kind measurement to apply */
void SunlineSRuKFAlgorithm::measurementUpdate(Measurement const& measurement) {
    std::visit([this](auto const& meas) { this->applyMeasurement(meas); }, measurement);
}

/*! Apply a CSS measurement and record its residuals.
 *  @return void
 *  @param measurement [-] packed CSS measurement (cosValues, H, noise) */
void SunlineSRuKFAlgorithm::applyMeasurement(CssMeasurement const& measurement) {
    CssMeasurementModel model;
    model.observed = measurement.cssCosValues;
    model.hMatrix = measurement.hMatrix;
    model.measNoise = measurement.covar;

    auto const result = this->srukf.measurementUpdate(model);

    this->lastCssResiduals.valid = measurement.valid;
    this->lastCssResiduals.numberOfActiveCss = measurement.numberOfActiveCss;
    this->lastCssResiduals.observation = model.observed;
    this->lastCssResiduals.preFit = result.preFit;
    this->lastCssResiduals.postFit = result.postFit;
}

/*! Apply a rate measurement and record its residuals.
 *  @return void
 *  @param measurement [-] packed rate measurement (omega, noise) */
void SunlineSRuKFAlgorithm::applyMeasurement(RateMeasurement const& measurement) {
    RateMeasurementModel model;
    model.observed = measurement.omega_BN_B;
    model.measNoise = measurement.covar;

    auto const result = this->srukf.measurementUpdate(model);

    this->lastRateResiduals.valid = measurement.valid;
    this->lastRateResiduals.observation = model.observed;
    this->lastRateResiduals.preFit = result.preFit;
    this->lastRateResiduals.postFit = result.postFit;
}

/*! Pack CSS readings into a CssMeasurement: active rows
 *  (cosValue > sensorUseThresh) populate the first `active` slots; H rows
 *  carry CBias * nHat vectors.
 *  @return CssMeasurement (valid = active > 0)
 *  @param cssData [-] raw CSS cos-values and time tag */
CssMeasurement SunlineSRuKFAlgorithm::packCssMeasurement(CssData const& cssData) const {
    double const cssMeasNoiseStd = this->cfg.getCssMeasurementNoiseStd();
    CssMeasurement packed;
    packed.timeTag = cssData.timeTag;
    packed.covar = (cssMeasNoiseStd * cssMeasNoiseStd) * Eigen::Matrix<double, MaxCss, MaxCss>::Identity();

    int active = 0;
    for (int i = 0; i < this->cfg.getNumberOfCss() && active < MaxCss; ++i) {
        if (cssData.cosValues(i) <= this->cfg.getSensorThreshold()) {
            continue;
        }
        packed.cssCosValues(active) = cssData.cosValues(i);
        packed.hMatrix.row(active) = this->cfg.getCssCBias()(i) * this->cfg.getCssNHat().row(i);
        active += 1;
    }
    packed.numberOfActiveCss = active;
    packed.valid = active > 0;
    return packed;
}

/*! Pack a raw gyro reading into a RateMeasurement with diagonal noise covar.
 *  @return RateMeasurement (always valid)
 *  @param rateData [-] raw rate vector and time tag */
RateMeasurement SunlineSRuKFAlgorithm::packRateMeasurement(RateData const& rateData) const {
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
SunlineSRuKFAlgorithm::State SunlineSRuKFAlgorithm::regularize(State const& state) const {
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
FilterStateOutput SunlineSRuKFAlgorithm::getFilterOutput() const {
    FilterStateOutput filterOutput;
    filterOutput.state = this->srukf.getState().raw();
    filterOutput.covariance = this->srukf.getCovariance();
    return filterOutput;
}

/*! @return const reference to the latest CSS residuals snapshot */
CssResidualsOutput const& SunlineSRuKFAlgorithm::getLastCssResiduals() const { return this->lastCssResiduals; }
/*! @return const reference to the latest rate residuals snapshot */
RateResidualsOutput const& SunlineSRuKFAlgorithm::getLastRateResiduals() const { return this->lastRateResiduals; }

/*! @return current filter state (post-sanitization) */
SunlineSRuKFAlgorithm::State SunlineSRuKFAlgorithm::getState() const { return this->srukf.getState(); }
/*! @return current full covariance */
Eigen::Matrix<double, SunlineSRuKFAlgorithm::N, SunlineSRuKFAlgorithm::N> SunlineSRuKFAlgorithm::getCovariance() const {
    return this->srukf.getCovariance();
}

}  // namespace filtering::sunlineSRuKF
