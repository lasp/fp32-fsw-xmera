#ifndef F32XMERA_FLYBYFILTERALGORITHM_H
#define F32XMERA_FLYBYFILTERALGORITHM_H

#include "flybyFilterSpecs.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/validPSDCheck.h"

#include <filteringCore/measurementQueue.h>
#include <filteringCore/kalmanFilter.hpp>
#include <filteringCore/srukf.hpp>

#include <Eigen/Core>

namespace filtering::flybyFilter {

/*! Raw optical-navigation heading reading consumed by FlybyFilterAlgorithm::update().
 *  `timeTag` > 0 flags that the adapter saw a fresh reading this cycle. */
struct HeadingData {
    double timeTag = 0;
    Eigen::Vector3d rhat_BN_N = Eigen::Vector3d::Zero();
};

/*! Snapshot returned by FlybyFilterAlgorithm::update(). Bundles the filter state with the
 *  heading residuals so the host adapter writes all output messages from one consistent
 *  post-update snapshot. */
struct FlybyFilterOutput {
    FilterStateOutput filterState;
    HeadingResidualsOutput headingResiduals;
};

/*! Validated, immutable configuration for FlybyFilterAlgorithm. create() validates every constrained
 *  parameter -- alpha/beta via the shared SRuKF validators, mu, the PSD process noise / initial
 *  covariance, and the heading measurement-noise std -- and throws on invalid input; the algorithm
 *  trusts the Config thereafter. All values are in internal units (the adapter converts from SI).
 *  Only initialState has no meaningful constraint. */
class FlybyFilterConfig final {
   public:
    // Reuse the framework SRuKF's static validators so filter-parameter checks live in one place.
    using Srukf = SRuKF<FlybyState, FlybyDynamics>;

    static FlybyFilterConfig create(double alpha,
                                    double beta,
                                    double mu,
                                    StateMatrix const& processNoise,
                                    FlybyState const& initialState,
                                    StateMatrix const& initialCovariance,
                                    double headingMeasurementNoiseStd,
                                    double outlierNSigma = 10.0) {
        if (!Srukf::alphaIsValid(alpha)) {
            FSW_THROW_INVALID_ARGUMENT("flybyFilter: alpha must be in (0, 1)");
        }
        if (!Srukf::betaIsValid(beta)) {
            FSW_THROW_INVALID_ARGUMENT("flybyFilter: beta must be in [0, 2]");
        }
        if (!isValidMu(mu)) {
            FSW_THROW_INVALID_ARGUMENT("flybyFilter: central-body gravitational parameter mu must be positive");
        }
        if (!isValidProcessNoise(processNoise)) {
            FSW_THROW_INVALID_ARGUMENT("flybyFilter: process noise must be positive semi-definite");
        }
        if (!isValidInitialCovariance(initialCovariance)) {
            FSW_THROW_INVALID_ARGUMENT("flybyFilter: initial covariance must be positive semi-definite");
        }
        if (!isValidHeadingMeasurementNoiseStd(headingMeasurementNoiseStd)) {
            FSW_THROW_INVALID_ARGUMENT("flybyFilter: heading measurement noise std must not be negative");
        }
        if (!isValidOutlierNSigma(outlierNSigma)) {
            FSW_THROW_INVALID_ARGUMENT("flybyFilter: outlier N-sigma must be greater than 0");
        }
        return {
            alpha, beta, mu, processNoise, initialState, initialCovariance, headingMeasurementNoiseStd, outlierNSigma};
    }

    static bool isValidMu(double mu) { return mu > 0.0; }
    static bool isValidProcessNoise(StateMatrix const& processNoise) {
        return isPositiveSemiDefinite<FlybyState::size>(processNoise);
    }
    static bool isValidInitialCovariance(StateMatrix const& covariance) {
        return isPositiveSemiDefinite<FlybyState::size>(covariance);
    }
    static bool isValidHeadingMeasurementNoiseStd(double noiseStd) { return noiseStd >= 0.0; }
    static bool isValidOutlierNSigma(double nSigma) { return nSigma > 0.0; }

    double getAlpha() const { return this->alpha; }
    double getBeta() const { return this->beta; }
    double getMu() const { return this->mu; }
    StateMatrix const& getProcessNoise() const { return this->processNoise; }
    FlybyState const& getInitialState() const { return this->initialState; }
    StateMatrix const& getInitialCovariance() const { return this->initialCovariance; }
    double getHeadingMeasurementNoiseStd() const { return this->headingMeasNoiseStd; }
    double getOutlierNSigma() const { return this->outlierNSigma; }

   private:
    FlybyFilterConfig(double alpha,
                      double beta,
                      double mu,
                      StateMatrix const& processNoise,
                      FlybyState const& initialState,
                      StateMatrix const& initialCovariance,
                      double headingMeasurementNoiseStd,
                      double outlierNSigma)
        : alpha(alpha),
          beta(beta),
          mu(mu),
          processNoise(processNoise),
          initialState(initialState),
          initialCovariance(initialCovariance),
          headingMeasNoiseStd(headingMeasurementNoiseStd),
          outlierNSigma(outlierNSigma) {}

    double alpha;
    double beta;
    double mu;
    StateMatrix processNoise;
    FlybyState initialState;
    StateMatrix initialCovariance;
    double headingMeasNoiseStd;
    double outlierNSigma;
};

/*! @brief Angles-only flyby navigation square-root UKF. Estimates the inertial spacecraft position
 *  and velocity relative to a central body from optical-navigation heading (unit-vector) measurements
 *  under two-body point-mass gravity. */
class FlybyFilterAlgorithm {
   public:
    using State = FlybyState;
    static constexpr int N = State::size;

    explicit FlybyFilterAlgorithm(const FlybyFilterConfig& config);

    void setConfig(FlybyFilterConfig const& config);

    FlybyFilterOutput update(double currentSeconds, HeadingData const& headingData);

    void reInitializeExceptPersistentStates();
    void reInitialize();
    bool timeUpdate(double dt);
    bool measurementUpdate(Measurement const& measurement);
    void clear();

    FilterStateOutput getFilterOutput() const;
    HeadingResidualsOutput const& getLastHeadingResiduals() const;
    State getState() const;
    Eigen::Matrix<double, N, N> getCovariance() const;

   private:
    bool applyMeasurement(HeadingMeasurement const& measurement);

    HeadingMeasurement packHeadingMeasurement(HeadingData const& headingData) const;

    FlybyFilterConfig cfg;  //!< validated configuration, supplied at construction / setConfig()
    SRuKF<State, FlybyDynamics> srukf;
    filtering::measurement_queue<Measurement, BatchSize> measurements;

    HeadingResidualsOutput
        lastHeadingResiduals{};  //!< latest heading residuals; valid=true only on cycles a measurement fired
};

}  // namespace filtering::flybyFilter

#endif
