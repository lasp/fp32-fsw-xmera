#ifndef F32XMERA_SUNLINEFILTERALGORITHM_H
#define F32XMERA_SUNLINEFILTERALGORITHM_H

#include "sunlineFilterSpecs.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/validPSDCheck.h"

#include <filteringCore/measurementQueue.h>
#include <filteringCore/kalmanFilter.hpp>
#include <filteringCore/srukf.hpp>

#include <math.h>

#include <Eigen/Core>
#include <cstdint>

namespace filtering::sunlineFilter {

/*! Raw CSS array reading consumed by SunlineFilterAlgorithm::update(). There
 *  is one `cosValues` slot per CSS; `timeTag` > 0 flags a fresh reading. */
struct CssData {
    double timeTag = 0;
    Eigen::Vector<float, MaxCss> cosValues = Eigen::Vector<float, MaxCss>::Zero();
};

/*! Raw gyro reading consumed by SunlineFilterAlgorithm::update(). `valid`
 *  flags whether the adapter saw a fresh reading this cycle. */
struct RateData {
    double timeTag = 0;
    Eigen::Vector3f rate = Eigen::Vector3f::Zero();
};

/*! Snapshot returned by SunlineFilterAlgorithm::update(). Bundles the filter
 *  state with the per-kind residuals so the host adapter writes all output
 *  messages from one consistent post-update snapshot. */
struct SunlineFilterOutput {
    FilterStateOutput filterState;
    CssResidualsOutput cssResiduals;
    RateResidualsOutput rateResiduals;
};

/*! Validated, immutable configuration for SunlineFilterAlgorithm. create() validates every constrained
 *  parameter -- alpha and beta through the shared SRuKF validators -- and normalizes the CSS boresights;
 *  the algorithm trusts the Config thereafter. The only parameter without a meaningful constraint
 *  (initialState) has no validator. */
class SunlineFilterConfig final {
   public:
    // Reuse the framework SRuKF's static validators so filter-parameter checks live in one place.
    using Srukf = SRuKF<SunlineState, SunlineDynamics>;

    static SunlineFilterConfig create(float alpha,
                                      float beta,
                                      StateMatrix const& processNoise,
                                      SunlineState const& initialState,
                                      StateMatrix const& initialCovariance,
                                      float biasLowerBound,
                                      float biasUpperBound,
                                      Eigen::Matrix<float, MaxCss, 3> const& cssNHat,
                                      Eigen::Vector<float, MaxCss> const& cssScaleFactor,
                                      float sensorThreshold,
                                      float cssMeasurementNoiseStd,
                                      float gyroMeasurementNoiseStd,
                                      float measurementDelay = 0.0F) {
        if (!Srukf::alphaAndBetaAreValid(alpha, beta)) {
            FSW_THROW_INVALID_ARGUMENT("sunlineFilter: alpha must be in (0, 1] and beta in [alpha^2, 2]");
        }
        if (!processNoise.allFinite()) {
            FSW_THROW_INVALID_ARGUMENT("sunlineFilter: process noise must be finite");
        }
        if (!isSymmetric<SunlineState::size>(processNoise)) {
            FSW_THROW_INVALID_ARGUMENT("sunlineFilter: process noise must be symmetric");
        }
        if (!isValidProcessNoise(processNoise)) {
            FSW_THROW_INVALID_ARGUMENT("sunlineFilter: process noise must be positive semi-definite");
        }
        if (!initialCovariance.allFinite()) {
            FSW_THROW_INVALID_ARGUMENT("sunlineFilter: initial covariance must be finite");
        }
        if (!isSymmetric<SunlineState::size>(initialCovariance)) {
            FSW_THROW_INVALID_ARGUMENT("sunlineFilter: initial covariance must be symmetric");
        }
        if (!isValidInitialCovariance(initialCovariance)) {
            FSW_THROW_INVALID_ARGUMENT("sunlineFilter: initial covariance must be positive semi-definite");
        }
        if (!isValidInitialState(initialState)) {
            FSW_THROW_INVALID_ARGUMENT("sunlineFilter: initial state must be finite");
        }
        if (!isValidBiasLowerBound(biasLowerBound)) {
            FSW_THROW_INVALID_ARGUMENT("sunlineFilter: bias lower bound must be greater than 0");
        }
        if (!isValidBiasUpperBound(biasUpperBound)) {
            FSW_THROW_INVALID_ARGUMENT("sunlineFilter: bias upper bound must be greater than 0");
        }
        if (!isValidBiasBounds(biasLowerBound, biasUpperBound)) {
            FSW_THROW_INVALID_ARGUMENT("sunlineFilter: bias lower bound must be less than bias upper bound");
        }
        if (!isValidCssNHat(cssNHat)) {
            FSW_THROW_INVALID_ARGUMENT("sunlineFilter: every CSS nHat row must be a unit vector within 1e-3");
        }
        if (!isValidCssScaleFactor(cssScaleFactor)) {
            FSW_THROW_INVALID_ARGUMENT("sunlineFilter: CSS calibration bias must not be negative");
        }
        if (!isValidSensorThreshold(sensorThreshold)) {
            FSW_THROW_INVALID_ARGUMENT("sunlineFilter: sensor threshold must not be negative");
        }
        if (!isValidCssMeasurementNoiseStd(cssMeasurementNoiseStd)) {
            FSW_THROW_INVALID_ARGUMENT(
                "sunlineFilter: CSS measurement noise std must be positive, with a finite square");
        }
        if (!isValidGyroMeasurementNoiseStd(gyroMeasurementNoiseStd)) {
            FSW_THROW_INVALID_ARGUMENT(
                "sunlineFilter: gyro measurement noise std must be positive, with a finite square");
        }
        if (measurementDelay < 0.0F) {
            FSW_THROW_INVALID_ARGUMENT("sunlineFilter: measurement delay must not be negative");
        }
        return {alpha,
                beta,
                processNoise,
                initialState,
                initialCovariance,
                biasLowerBound,
                biasUpperBound,
                normalizeCssNHat(cssNHat),
                cssScaleFactor,
                sensorThreshold,
                cssMeasurementNoiseStd,
                gyroMeasurementNoiseStd,
                measurementDelay};
    }

    static bool isValidProcessNoise(StateMatrix const& processNoise) {
        return processNoise.allFinite() && isSymmetric<SunlineState::size>(processNoise) &&
               isPositiveSemiDefinite<SunlineState::size>(processNoise);
    }
    static bool isValidInitialState(SunlineState const& initialState) { return initialState.allFinite(); }
    static bool isValidInitialCovariance(StateMatrix const& covariance) {
        return covariance.allFinite() && isSymmetric<SunlineState::size>(covariance) &&
               isPositiveSemiDefinite<SunlineState::size>(covariance);
    }
    static bool isValidBiasLowerBound(float bound) { return bound > 0.0F; }
    static bool isValidBiasUpperBound(float bound) { return bound > 0.0F; }
    static bool isValidBiasBounds(float lowerBound, float upperBound) { return lowerBound < upperBound; }
    static bool isValidCssNHat(Eigen::Matrix<float, MaxCss, 3> const& cssNHat) {
        constexpr float normTolerance = 1e-3F;
        for (uint32_t i = 0; i < MaxCss; ++i) {
            Eigen::Vector3f const row = cssNHat.row(i).transpose();
            if (fabsf(row.stableNorm() - 1.0F) > normTolerance) {
                return false;
            }
        }
        return true;
    }
    static bool isValidCssScaleFactor(Eigen::Vector<float, MaxCss> const& cssScaleFactor) {
        return (cssScaleFactor.array() >= 0.0F).all();
    }
    static bool isValidSensorThreshold(float threshold) { return threshold >= 0.0F; }
    static bool isValidCssMeasurementNoiseStd(float noiseStd) { return Srukf::measurementNoiseStdIsValid(noiseStd); }
    static bool isValidGyroMeasurementNoiseStd(float noiseStd) { return Srukf::measurementNoiseStdIsValid(noiseStd); }

    float getAlpha() const { return this->alpha; }
    float getBeta() const { return this->beta; }
    StateMatrix const& getProcessNoise() const { return this->processNoise; }
    SunlineState const& getInitialState() const { return this->initialState; }
    StateMatrix const& getInitialCovariance() const { return this->initialCovariance; }
    float getBiasLowerBound() const { return this->biasLowerBound; }
    float getBiasUpperBound() const { return this->biasUpperBound; }
    Eigen::Matrix<float, MaxCss, 3> const& getCssNHat() const { return this->cssNHat; }
    Eigen::Vector<float, MaxCss> const& getCssScaleFactor() const { return this->cssScaleFactor; }
    float getSensorThreshold() const { return this->sensorThreshold; }
    float getCssMeasurementNoiseStd() const { return this->cssMeasNoiseStd; }
    float getGyroMeasurementNoiseStd() const { return this->gyroMeasNoiseStd; }
    float getMeasurementDelay() const { return this->measurementDelay; }

   private:
    SunlineFilterConfig(float alpha,
                        float beta,
                        StateMatrix const& processNoise,
                        SunlineState const& initialState,
                        StateMatrix const& initialCovariance,
                        float biasLowerBound,
                        float biasUpperBound,
                        Eigen::Matrix<float, MaxCss, 3> const& cssNHat,
                        Eigen::Vector<float, MaxCss> const& cssScaleFactor,
                        float sensorThreshold,
                        float cssMeasurementNoiseStd,
                        float gyroMeasurementNoiseStd,
                        float measurementDelay)
        : alpha(alpha),
          beta(beta),
          processNoise(processNoise),
          initialState(initialState),
          initialCovariance(initialCovariance),
          biasLowerBound(biasLowerBound),
          biasUpperBound(biasUpperBound),
          cssNHat(cssNHat),
          cssScaleFactor(cssScaleFactor),
          sensorThreshold(sensorThreshold),
          cssMeasNoiseStd(cssMeasurementNoiseStd),
          gyroMeasNoiseStd(gyroMeasurementNoiseStd),
          measurementDelay(measurementDelay) {}

    static Eigen::Matrix<float, MaxCss, 3> normalizeCssNHat(Eigen::Matrix<float, MaxCss, 3> const& cssNHat) {
        Eigen::Matrix<float, MaxCss, 3> normalized = Eigen::Matrix<float, MaxCss, 3>::Zero();
        for (uint32_t i = 0; i < MaxCss; ++i) {
            Eigen::Vector3f const row = cssNHat.row(i).transpose();
            normalized.row(i) = row.stableNormalized().transpose();
        }
        return normalized;
    }

    float alpha;
    float beta;
    StateMatrix processNoise;
    SunlineState initialState;
    StateMatrix initialCovariance;
    float biasLowerBound;
    float biasUpperBound;
    Eigen::Matrix<float, MaxCss, 3> cssNHat;
    Eigen::Vector<float, MaxCss> cssScaleFactor;
    float sensorThreshold;
    float cssMeasNoiseStd;
    float gyroMeasNoiseStd;
    float measurementDelay;
};

/*! @brief Sunline square-root UKF. Estimates sun-heading direction, body rate,
 *  and a CSS bias from CSS array measurements and gyro rates on one timeline. */
class SunlineFilterAlgorithm {
   public:
    using State = SunlineState;
    static constexpr int N = State::size;

    explicit SunlineFilterAlgorithm(const SunlineFilterConfig& config);

    void setConfig(SunlineFilterConfig const& config);

    SunlineFilterOutput update(double currentSeconds, CssData const& cssData, RateData const& rateData);

    void reInitializeExceptPersistentStates();
    void reInitialize();
    bool timeUpdate(float dt);
    bool measurementUpdate(Measurement const& measurement);
    void clear();

    FilterStateOutput getFilterOutput() const;
    CssResidualsOutput const& getLastCssResiduals() const;
    RateResidualsOutput const& getLastRateResiduals() const;
    State getState() const;
    Eigen::Matrix<float, N, N> getCovariance() const;

   private:
    bool applyMeasurement(CssMeasurement const& measurement);
    bool applyMeasurement(RateMeasurement const& measurement);

    CssMeasurement packCssMeasurement(CssData const& cssData) const;
    RateMeasurement packRateMeasurement(RateData const& rateData) const;

    State regularize(State const& state) const;

    SunlineFilterConfig cfg;  //!< validated configuration, supplied at construction / setConfig()
    SRuKF<State, SunlineDynamics> srukf;
    filtering::measurement_queue<Measurement, BatchSize> measurements;

    CssResidualsOutput lastCssResiduals{};  //!< latest CSS residuals; valid=true only on cycles a CSS measurement fired
    RateResidualsOutput
        lastRateResiduals{};  //!< latest rate residuals; valid=true only on cycles a rate measurement fired
};

}  // namespace filtering::sunlineFilter

#endif
