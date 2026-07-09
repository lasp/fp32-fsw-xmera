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

/*! Raw CSS array reading consumed by SunlineFilterAlgorithm::update(). The
 *  first `numberOfCss` slots of `cosValues` are meaningful; `valid` flags
 *  whether the adapter saw a fresh reading this cycle. */
struct CssData {
    double timeTag = 0;
    Eigen::Vector<double, MaxCss> cosValues = Eigen::Vector<double, MaxCss>::Zero();
};

/*! Raw gyro reading consumed by SunlineFilterAlgorithm::update(). `valid`
 *  flags whether the adapter saw a fresh reading this cycle. */
struct RateData {
    double timeTag = 0;
    Eigen::Vector3d rate = Eigen::Vector3d::Zero();
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

    static SunlineFilterConfig create(double alpha,
                                      double beta,
                                      StateMatrix const& processNoise,
                                      SunlineState const& initialState,
                                      StateMatrix const& initialCovariance,
                                      double biasLowerBound,
                                      double biasUpperBound,
                                      Eigen::Matrix<double, MaxCss, 3> const& cssNHat,
                                      Eigen::Vector<double, MaxCss> const& cssScaleFactor,
                                      uint32_t numberOfCss,
                                      double sensorThreshold,
                                      double cssMeasurementNoiseStd,
                                      double gyroMeasurementNoiseStd) {
        if (!Srukf::alphaIsValid(alpha)) {
            FSW_THROW_INVALID_ARGUMENT("sunlineFilter: alpha must be in (0, 1)");
        }
        if (!Srukf::betaIsValid(beta)) {
            FSW_THROW_INVALID_ARGUMENT("sunlineFilter: beta must be in [0, 2]");
        }
        if (!isValidProcessNoise(processNoise)) {
            FSW_THROW_INVALID_ARGUMENT("sunlineFilter: process noise must be positive semi-definite");
        }
        if (!isValidInitialCovariance(initialCovariance)) {
            FSW_THROW_INVALID_ARGUMENT("sunlineFilter: initial covariance must be positive semi-definite");
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
        if (!isValidNumberOfCss(numberOfCss)) {
            FSW_THROW_INVALID_ARGUMENT("sunlineFilter: numberOfCss must be in [1, MaxCss]");
        }
        if (!isValidCssNHat(cssNHat, numberOfCss)) {
            FSW_THROW_INVALID_ARGUMENT(
                "sunlineFilter: the first numberOfCss CSS nHat rows must be unit vectors within 1e-3");
        }
        if (!isValidCssScaleFactor(cssScaleFactor)) {
            FSW_THROW_INVALID_ARGUMENT("sunlineFilter: CSS calibration bias must not be negative");
        }
        if (!isValidSensorThreshold(sensorThreshold)) {
            FSW_THROW_INVALID_ARGUMENT("sunlineFilter: sensor threshold must not be negative");
        }
        if (!isValidCssMeasurementNoiseStd(cssMeasurementNoiseStd)) {
            FSW_THROW_INVALID_ARGUMENT("sunlineFilter: CSS measurement noise std must not be negative");
        }
        if (!isValidGyroMeasurementNoiseStd(gyroMeasurementNoiseStd)) {
            FSW_THROW_INVALID_ARGUMENT("sunlineFilter: gyro measurement noise std must not be negative");
        }
        return {alpha,
                beta,
                processNoise,
                initialState,
                initialCovariance,
                biasLowerBound,
                biasUpperBound,
                normalizeCssNHat(cssNHat, numberOfCss),
                cssScaleFactor,
                numberOfCss,
                sensorThreshold,
                cssMeasurementNoiseStd,
                gyroMeasurementNoiseStd};
    }

    static bool isValidProcessNoise(StateMatrix const& processNoise) {
        return isPositiveSemiDefinite<SunlineState::size>(processNoise);
    }
    static bool isValidInitialCovariance(StateMatrix const& covariance) {
        return isPositiveSemiDefinite<SunlineState::size>(covariance);
    }
    static bool isValidBiasLowerBound(double bound) { return bound > 0.0; }
    static bool isValidBiasUpperBound(double bound) { return bound > 0.0; }
    static bool isValidBiasBounds(double lowerBound, double upperBound) { return lowerBound < upperBound; }
    static bool isValidCssNHat(Eigen::Matrix<double, MaxCss, 3> const& cssNHat, uint32_t numberOfCss) {
        constexpr double normTolerance = 1e-3;
        for (uint32_t i = 0; i < numberOfCss; ++i) {
            Eigen::Vector3d const row = cssNHat.row(i).transpose();
            if (fabs(row.stableNorm() - 1.0) > normTolerance) {
                return false;
            }
        }
        return true;
    }
    static bool isValidCssScaleFactor(Eigen::Vector<double, MaxCss> const& cssScaleFactor) {
        return (cssScaleFactor.array() >= 0.0).all();
    }
    static bool isValidNumberOfCss(uint32_t count) { return count > 0 && count <= static_cast<uint32_t>(MaxCss); }
    static bool isValidSensorThreshold(double threshold) { return threshold >= 0.0; }
    static bool isValidCssMeasurementNoiseStd(double noiseStd) { return noiseStd >= 0.0; }
    static bool isValidGyroMeasurementNoiseStd(double noiseStd) { return noiseStd >= 0.0; }

    double getAlpha() const { return this->alpha; }
    double getBeta() const { return this->beta; }
    StateMatrix const& getProcessNoise() const { return this->processNoise; }
    SunlineState const& getInitialState() const { return this->initialState; }
    StateMatrix const& getInitialCovariance() const { return this->initialCovariance; }
    double getBiasLowerBound() const { return this->biasLowerBound; }
    double getBiasUpperBound() const { return this->biasUpperBound; }
    Eigen::Matrix<double, MaxCss, 3> const& getCssNHat() const { return this->cssNHat; }
    Eigen::Vector<double, MaxCss> const& getCssScaleFactor() const { return this->cssScaleFactor; }
    uint32_t getNumberOfCss() const { return this->numberOfCss; }
    double getSensorThreshold() const { return this->sensorThreshold; }
    double getCssMeasurementNoiseStd() const { return this->cssMeasNoiseStd; }
    double getGyroMeasurementNoiseStd() const { return this->gyroMeasNoiseStd; }

   private:
    SunlineFilterConfig(double alpha,
                        double beta,
                        StateMatrix const& processNoise,
                        SunlineState const& initialState,
                        StateMatrix const& initialCovariance,
                        double biasLowerBound,
                        double biasUpperBound,
                        Eigen::Matrix<double, MaxCss, 3> const& cssNHat,
                        Eigen::Vector<double, MaxCss> const& cssScaleFactor,
                        uint32_t numberOfCss,
                        double sensorThreshold,
                        double cssMeasurementNoiseStd,
                        double gyroMeasurementNoiseStd)
        : alpha(alpha),
          beta(beta),
          processNoise(processNoise),
          initialState(initialState),
          initialCovariance(initialCovariance),
          biasLowerBound(biasLowerBound),
          biasUpperBound(biasUpperBound),
          cssNHat(cssNHat),
          cssScaleFactor(cssScaleFactor),
          numberOfCss(numberOfCss),
          sensorThreshold(sensorThreshold),
          cssMeasNoiseStd(cssMeasurementNoiseStd),
          gyroMeasNoiseStd(gyroMeasurementNoiseStd) {}

    static Eigen::Matrix<double, MaxCss, 3> normalizeCssNHat(Eigen::Matrix<double, MaxCss, 3> const& cssNHat,
                                                             uint32_t numberOfCss) {
        Eigen::Matrix<double, MaxCss, 3> normalized = Eigen::Matrix<double, MaxCss, 3>::Zero();
        for (uint32_t i = 0; i < numberOfCss; ++i) {
            Eigen::Vector3d const row = cssNHat.row(i).transpose();
            normalized.row(i) = row.stableNormalized().transpose();
        }
        return normalized;
    }

    double alpha;
    double beta;
    StateMatrix processNoise;
    SunlineState initialState;
    StateMatrix initialCovariance;
    double biasLowerBound;
    double biasUpperBound;
    Eigen::Matrix<double, MaxCss, 3> cssNHat;
    Eigen::Vector<double, MaxCss> cssScaleFactor;
    uint32_t numberOfCss;
    double sensorThreshold;
    double cssMeasNoiseStd;
    double gyroMeasNoiseStd;
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

    void reInitialize();
    void reInitializeAll();
    bool timeUpdate(double dt);
    bool measurementUpdate(Measurement const& measurement);
    void clear();

    FilterStateOutput getFilterOutput() const;
    CssResidualsOutput const& getLastCssResiduals() const;
    RateResidualsOutput const& getLastRateResiduals() const;
    State getState() const;
    Eigen::Matrix<double, N, N> getCovariance() const;

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
