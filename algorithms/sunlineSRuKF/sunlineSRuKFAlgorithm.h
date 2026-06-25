#ifndef F32XMERA_SUNLINESRUKFALGORITHM_H
#define F32XMERA_SUNLINESRUKFALGORITHM_H

#include "sunlineSRuKFSpecs.h"
#include "utilities/freestandingInvalidArgument.h"
#include "utilities/validPSDCheck.h"

#include <filteringCore/measurementQueue.h>
#include <filteringCore/kalmanFilter.hpp>
#include <filteringCore/srukf.hpp>

#include <math.h>

#include <Eigen/Core>

namespace filtering::sunlineSRuKF {

/*! Raw CSS array reading consumed by SunlineSRuKFAlgorithm::update(). The
 *  first `numberOfCss` slots of `cosValues` are meaningful; `valid` flags
 *  whether the adapter saw a fresh reading this cycle. */
struct CssData {
    double timeTag = 0;
    Eigen::Vector<double, MaxCss> cosValues = Eigen::Vector<double, MaxCss>::Zero();
};

/*! Raw gyro reading consumed by SunlineSRuKFAlgorithm::update(). `valid`
 *  flags whether the adapter saw a fresh reading this cycle. */
struct RateData {
    double timeTag = 0;
    Eigen::Vector3d rate = Eigen::Vector3d::Zero();
};

/*! Snapshot returned by SunlineSRuKFAlgorithm::update(). Bundles the filter
 *  state with the per-kind residuals so the host adapter writes all output
 *  messages from one consistent post-update snapshot. */
struct SunlineSRuKFOutput {
    FilterStateOutput filterState;
    CssResidualsOutput cssResiduals;
    RateResidualsOutput rateResiduals;
};

/*! Validated, immutable configuration for SunlineSRuKFAlgorithm. create() validates every constrained
 *  parameter and normalizes the CSS boresights; the algorithm trusts the Config thereafter. Parameters
 *  without a meaningful constraint (alpha, beta, initialState) have no validator. */
class SunlineSRuKFConfig final {
   public:
    static SunlineSRuKFConfig create(double alpha,
                                     double beta,
                                     StateMatrix const& processNoise,
                                     SunlineState const& initialState,
                                     StateMatrix const& initialCovariance,
                                     double biasLowerBound,
                                     double biasUpperBound,
                                     Eigen::Matrix<double, MaxCss, 3> const& cssNHat,
                                     Eigen::Vector<double, MaxCss> const& cssScaleFactor,
                                     int numberOfCss,
                                     double sensorThreshold,
                                     double cssMeasurementNoiseStd,
                                     double gyroMeasurementNoiseStd) {
        if (!isValidProcessNoise(processNoise)) {
            FSW_THROW_INVALID_ARGUMENT("sunlineSRuKF: process noise must be positive semi-definite");
        }
        if (!isValidInitialCovariance(initialCovariance)) {
            FSW_THROW_INVALID_ARGUMENT("sunlineSRuKF: initial covariance must be positive semi-definite");
        }
        if (!isValidBiasLowerBound(biasLowerBound)) {
            FSW_THROW_INVALID_ARGUMENT("sunlineSRuKF: bias lower bound must be greater than 0");
        }
        if (!isValidBiasUpperBound(biasUpperBound)) {
            FSW_THROW_INVALID_ARGUMENT("sunlineSRuKF: bias upper bound must be greater than 0");
        }
        if (!isValidBiasBounds(biasLowerBound, biasUpperBound)) {
            FSW_THROW_INVALID_ARGUMENT("sunlineSRuKF: bias lower bound must be less than bias upper bound");
        }
        if (!isValidNumberOfCss(numberOfCss)) {
            FSW_THROW_INVALID_ARGUMENT("sunlineSRuKF: numberOfCss must be in [0, MaxCss]");
        }
        if (!isValidCssNHat(cssNHat, numberOfCss)) {
            FSW_THROW_INVALID_ARGUMENT(
                "sunlineSRuKF: the first numberOfCss CSS nHat rows must be unit vectors within 1e-3");
        }
        if (!isValidCssScaleFactor(cssScaleFactor)) {
            FSW_THROW_INVALID_ARGUMENT("sunlineSRuKF: CSS calibration bias must not be negative");
        }
        if (!isValidSensorThreshold(sensorThreshold)) {
            FSW_THROW_INVALID_ARGUMENT("sunlineSRuKF: sensor threshold must not be negative");
        }
        if (!isValidCssMeasurementNoiseStd(cssMeasurementNoiseStd)) {
            FSW_THROW_INVALID_ARGUMENT("sunlineSRuKF: CSS measurement noise std must not be negative");
        }
        if (!isValidGyroMeasurementNoiseStd(gyroMeasurementNoiseStd)) {
            FSW_THROW_INVALID_ARGUMENT("sunlineSRuKF: gyro measurement noise std must not be negative");
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
    static bool isValidCssNHat(Eigen::Matrix<double, MaxCss, 3> const& cssNHat, int numberOfCss) {
        constexpr double normTolerance = 1e-3;
        for (int i = 0; i < numberOfCss; ++i) {
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
    static bool isValidNumberOfCss(int count) { return count >= 0 && count <= MaxCss; }
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
    int getNumberOfCss() const { return this->numberOfCss; }
    double getSensorThreshold() const { return this->sensorThreshold; }
    double getCssMeasurementNoiseStd() const { return this->cssMeasNoiseStd; }
    double getGyroMeasurementNoiseStd() const { return this->gyroMeasNoiseStd; }

   private:
    SunlineSRuKFConfig(double alpha,
                       double beta,
                       StateMatrix const& processNoise,
                       SunlineState const& initialState,
                       StateMatrix const& initialCovariance,
                       double biasLowerBound,
                       double biasUpperBound,
                       Eigen::Matrix<double, MaxCss, 3> const& cssNHat,
                       Eigen::Vector<double, MaxCss> const& cssScaleFactor,
                       int numberOfCss,
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
                                                             int numberOfCss) {
        Eigen::Matrix<double, MaxCss, 3> normalized = Eigen::Matrix<double, MaxCss, 3>::Zero();
        for (int i = 0; i < numberOfCss; ++i) {
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
    int numberOfCss;
    double sensorThreshold;
    double cssMeasNoiseStd;
    double gyroMeasNoiseStd;
};

/*! @brief Sunline square-root UKF. Estimates sun-heading direction, body rate,
 *  and a CSS bias from CSS array measurements and gyro rates on one timeline. */
class SunlineSRuKFAlgorithm {
   public:
    using State = SunlineState;
    static constexpr int N = State::size;

    explicit SunlineSRuKFAlgorithm(const SunlineSRuKFConfig& config);

    void setConfig(SunlineSRuKFConfig const& config);

    SunlineSRuKFOutput update(double currentSeconds, CssData const& cssData, RateData const& rateData);

    void reInitialize();
    void reInitializeAll();
    void timeUpdate(double dt);
    void measurementUpdate(Measurement const& measurement);

    FilterStateOutput getFilterOutput() const;
    CssResidualsOutput const& getLastCssResiduals() const;
    RateResidualsOutput const& getLastRateResiduals() const;
    State getState() const;
    Eigen::Matrix<double, N, N> getCovariance() const;

   private:
    void applyMeasurement(CssMeasurement const& measurement);
    void applyMeasurement(RateMeasurement const& measurement);

    CssMeasurement packCssMeasurement(CssData const& cssData) const;
    RateMeasurement packRateMeasurement(RateData const& rateData) const;

    State regularize(State const& state) const;

    SunlineSRuKFConfig cfg;  //!< validated configuration, supplied at construction / setConfig()
    SRuKF<State, SunlineDynamics> srukf;
    filtering::measurement_queue<Measurement, BatchSize> measurements;

    CssResidualsOutput lastCssResiduals{};  //!< latest CSS residuals; valid=true only on cycles a CSS measurement fired
    RateResidualsOutput
        lastRateResiduals{};  //!< latest rate residuals; valid=true only on cycles a rate measurement fired
};

}  // namespace filtering::sunlineSRuKF

#endif
