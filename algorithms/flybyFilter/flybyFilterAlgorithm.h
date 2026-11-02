#ifndef F32XMERA_FLYBYFILTERALGORITHM_H
#define F32XMERA_FLYBYFILTERALGORITHM_H

#include "flybyFilterSpecs.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/freestandingIsFinite.hpp"
#include "utilities/fsw/validPSDCheck.h"

#include <filteringCore/measurementQueue.h>
#include <filteringCore/kalmanFilter.hpp>
#include <filteringCore/srukf.hpp>

#include <Eigen/Core>

#include <math.h>

namespace filtering::flybyFilter {

/*! Raw optical-navigation heading reading consumed by FlybyFilterAlgorithm::update().
 *  `timeTag` > 0 flags that the adapter saw a fresh reading this cycle. `covar_N` is the heading
 *  noise covariance supplied with the reading (e.g. by cobConverter); a reading whose covariance is
 *  not finite and positive definite is dropped, so the zero default never reaches the filter. `shat_N`
 *  is the sun direction at the reading's time (e.g. from navAggregate's attitude output), which defines the
 *  terminator directions of the heading bias; with the zero default, or a non-finite value, the reading is
 *  modeled without the bias. */
struct HeadingData {
    double timeTag = 0;
    Eigen::Vector3f rhat_BN_N = Eigen::Vector3f::Zero();
    Eigen::Matrix3f covar_N = Eigen::Matrix3f::Zero();
    Eigen::Vector3f shat_N = Eigen::Vector3f::Zero();
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
 *  covariance, the measurement-noise scale, the heading-bias maxima and the initial bias within them --
 *  and throws on invalid input; the algorithm trusts the Config thereafter. All values are in internal
 *  units (the adapter converts from SI). */
class FlybyFilterConfig final {
   public:
    // Reuse the framework SRuKF's static validators so filter-parameter checks live in one place.
    using Srukf = SRuKF<FlybyState, FlybyDynamics>;

    static FlybyFilterConfig create(float alpha,
                                    float beta,
                                    float mu,
                                    StateMatrix const& processNoise,
                                    FlybyState const& initialState,
                                    StateMatrix const& initialCovariance,
                                    float measurementNoiseScale,
                                    float maxAcrossTerminatorBias,
                                    float maxAlongTerminatorBias,
                                    float measurementDelay = 0.0F) {
        if (!Srukf::alphaAndBetaAreValid(alpha, beta)) {
            FSW_THROW_INVALID_ARGUMENT("flybyFilter: alpha must be in (0, 1] and beta in [alpha^2, 2]");
        }
        if (!isValidMu(mu)) {
            FSW_THROW_INVALID_ARGUMENT("flybyFilter: central-body gravitational parameter mu must be positive");
        }
        if (!processNoise.allFinite()) {
            FSW_THROW_INVALID_ARGUMENT("flybyFilter: process noise must be finite");
        }
        if (!isSymmetric<FlybyState::size>(processNoise)) {
            FSW_THROW_INVALID_ARGUMENT("flybyFilter: process noise must be symmetric");
        }
        if (!isValidProcessNoise(processNoise)) {
            FSW_THROW_INVALID_ARGUMENT("flybyFilter: process noise must be positive semi-definite");
        }
        if (!initialCovariance.allFinite()) {
            FSW_THROW_INVALID_ARGUMENT("flybyFilter: initial covariance must be finite");
        }
        if (!isSymmetric<FlybyState::size>(initialCovariance)) {
            FSW_THROW_INVALID_ARGUMENT("flybyFilter: initial covariance must be symmetric");
        }
        if (!isValidInitialCovariance(initialCovariance)) {
            FSW_THROW_INVALID_ARGUMENT("flybyFilter: initial covariance must be positive semi-definite");
        }
        if (!isValidInitialState(initialState)) {
            FSW_THROW_INVALID_ARGUMENT("flybyFilter: initial state must be finite");
        }
        if (!isValidMeasurementNoiseScale(measurementNoiseScale)) {
            FSW_THROW_INVALID_ARGUMENT("flybyFilter: measurement noise scale must be finite and positive");
        }
        if (!isValidMaxBias(maxAcrossTerminatorBias)) {
            FSW_THROW_INVALID_ARGUMENT("flybyFilter: maximum across-terminator bias must be finite and positive");
        }
        if (!isValidMaxBias(maxAlongTerminatorBias)) {
            FSW_THROW_INVALID_ARGUMENT("flybyFilter: maximum along-terminator bias must be finite and positive");
        }
        if (!biasIsWithinBounds(
                initialState.get<filtering::Bias<2>>(), maxAcrossTerminatorBias, maxAlongTerminatorBias)) {
            FSW_THROW_INVALID_ARGUMENT("flybyFilter: initial heading bias must be within its maxima");
        }
        if (measurementDelay < 0.0F) {
            FSW_THROW_INVALID_ARGUMENT("flybyFilter: measurement delay must not be negative");
        }
        return {alpha,
                beta,
                mu,
                processNoise,
                initialState,
                initialCovariance,
                measurementNoiseScale,
                maxAcrossTerminatorBias,
                maxAlongTerminatorBias,
                measurementDelay};
    }

    static bool isValidMu(float mu) { return mu > 0.0F; }
    static bool isValidProcessNoise(StateMatrix const& processNoise) {
        return processNoise.allFinite() && isSymmetric<FlybyState::size>(processNoise) &&
               isPositiveSemiDefinite<FlybyState::size>(processNoise);
    }
    static bool isValidInitialState(FlybyState const& initialState) { return initialState.allFinite(); }
    static bool isValidInitialCovariance(StateMatrix const& covariance) {
        return covariance.allFinite() && isSymmetric<FlybyState::size>(covariance) &&
               isPositiveSemiDefinite<FlybyState::size>(covariance);
    }
    static bool isValidMeasurementNoiseScale(float scale) { return fsw::is_finite(scale) && scale > 0.0F; }
    static bool isValidMaxBias(float maxBias) { return fsw::is_finite(maxBias) && maxBias > 0.0F; }
    /*! @return true iff each bias angle is within its maximum magnitude (false for a NaN angle)
     *  @param bias     [rad] {across, along} terminator heading-bias angles
     *  @param maxAcross [rad] maximum across-terminator angle magnitude
     *  @param maxAlong  [rad] maximum along-terminator angle magnitude */
    static bool biasIsWithinBounds(Eigen::Vector2f const& bias, float maxAcross, float maxAlong) {
        return fabsf(bias(AcrossTerminator)) <= maxAcross && fabsf(bias(AlongTerminator)) <= maxAlong;
    }

    float getAlpha() const { return this->alpha; }
    float getBeta() const { return this->beta; }
    float getMu() const { return this->mu; }
    StateMatrix const& getProcessNoise() const { return this->processNoise; }
    FlybyState const& getInitialState() const { return this->initialState; }
    StateMatrix const& getInitialCovariance() const { return this->initialCovariance; }
    float getMeasurementNoiseScale() const { return this->measNoiseScale; }
    float getMaxAcrossTerminatorBias() const { return this->maxAcrossTerminatorBias; }
    float getMaxAlongTerminatorBias() const { return this->maxAlongTerminatorBias; }
    float getMeasurementDelay() const { return this->measurementDelay; }

   private:
    FlybyFilterConfig(float alpha,
                      float beta,
                      float mu,
                      StateMatrix const& processNoise,
                      FlybyState const& initialState,
                      StateMatrix const& initialCovariance,
                      float measurementNoiseScale,
                      float maxAcrossTerminatorBias,
                      float maxAlongTerminatorBias,
                      float measurementDelay)
        : alpha(alpha),
          beta(beta),
          mu(mu),
          processNoise(processNoise),
          initialState(initialState),
          initialCovariance(initialCovariance),
          measNoiseScale(measurementNoiseScale),
          maxAcrossTerminatorBias(maxAcrossTerminatorBias),
          maxAlongTerminatorBias(maxAlongTerminatorBias),
          measurementDelay(measurementDelay) {}

    float alpha;
    float beta;
    float mu;
    StateMatrix processNoise;
    FlybyState initialState;
    StateMatrix initialCovariance;
    float measNoiseScale;           //!< [-] multiplies each reading's covar_N to form the measurement noise
    float maxAcrossTerminatorBias;  //!< [rad] across-terminator bias magnitude above which the bias is re-seeded
    float maxAlongTerminatorBias;   //!< [rad] along-terminator bias magnitude above which the bias is re-seeded
    float measurementDelay;
};

/*! @brief Angles-only flyby navigation square-root UKF. Estimates the inertial spacecraft position
 *  and velocity relative to a central body, and a constant two-angle heading bias across and along the
 *  terminator, from optical-navigation heading (unit-vector) measurements under two-body point-mass gravity. */
class FlybyFilterAlgorithm {
   public:
    using State = FlybyState;
    static constexpr int N = State::size;

    explicit FlybyFilterAlgorithm(const FlybyFilterConfig& config);

    void setConfig(FlybyFilterConfig const& config);

    FlybyFilterOutput update(double currentSeconds, HeadingData const& headingData);

    void reInitializeExceptPersistentStates();
    void reInitialize();
    bool timeUpdate(float dt);
    bool measurementUpdate(Measurement const& measurement);
    void clear();

    FilterStateOutput getFilterOutput() const;
    HeadingResidualsOutput const& getLastHeadingResiduals() const;
    State getState() const;
    Eigen::Matrix<float, N, N> getCovariance() const;

   private:
    bool applyMeasurement(HeadingMeasurement const& measurement);
    void resetBiasOutsideBounds();

    HeadingMeasurement packHeadingMeasurement(HeadingData const& headingData) const;

    FlybyFilterConfig cfg;  //!< validated configuration, supplied at construction / setConfig()
    SRuKF<State, FlybyDynamics> srukf;
    filtering::measurement_queue<Measurement, BatchSize> measurements;

    HeadingResidualsOutput
        lastHeadingResiduals{};  //!< latest heading residuals; valid=true only on cycles a measurement fired
};

}  // namespace filtering::flybyFilter

#endif
