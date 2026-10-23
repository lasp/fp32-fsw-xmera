#ifndef F32XMERA_INERTIALFILTERALGORITHM_H
#define F32XMERA_INERTIALFILTERALGORITHM_H

#include "inertialFilterSpecs.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/validPSDCheck.h"

#include <filteringCore/measurementQueue.h>
#include <filteringCore/kalmanFilter.hpp>
#include <filteringCore/srukf.hpp>

#include <math.h>

#include <Eigen/Core>

namespace filtering::inertialFilter {

/*! Raw star-tracker attitude reading consumed by InertialFilterAlgorithm::update().
 *  `timeTag` > 0 flags that the adapter saw a fresh reading this cycle. */
struct StAttData {
    double timeTag = 0;
    Eigen::Vector3f sigma_BN = Eigen::Vector3f::Zero();
};

/*! Raw rate reading consumed by InertialFilterAlgorithm::update(). `timeTag` > 0
 *  flags that the adapter saw a fresh reading this cycle. */
struct RateData {
    double timeTag = 0;
    Eigen::Vector3f rate = Eigen::Vector3f::Zero();
};

/*! Snapshot returned by InertialFilterAlgorithm::update(). Bundles the filter
 *  state with the per-kind residuals so the host adapter writes all output
 *  messages from one consistent post-update snapshot. */
struct InertialFilterOutput {
    FilterStateOutput filterState;
    StAttResidualsOutput stAttResiduals;
    RateResidualsOutput rateResiduals;
};

/*! Validated configuration for InertialFilterAlgorithm. create() validates every constrained
 *  parameter through the shared SRuKF validators (alpha, beta), coariance and process noise, and
 *  the inertial-specific measurement-noise checks. */
class InertialFilterConfig final {
   public:
    // Reuse the framework SRuKF's static validators so filter-parameter checks live in one place.
    using Srukf = SRuKF<InertialState, InertialDynamics>;

    static InertialFilterConfig create(float alpha,
                                       float beta,
                                       StateMatrix const& processNoise,
                                       InertialState const& initialState,
                                       StateMatrix const& initialCovariance,
                                       float stMeasurementNoiseStd,
                                       float rateMeasurementNoiseStd) {
        if (!Srukf::alphaIsValid(alpha)) {
            FSW_THROW_INVALID_ARGUMENT("inertialFilter: alpha must be in (0, 1]");
        }
        if (!Srukf::betaIsValid(beta)) {
            FSW_THROW_INVALID_ARGUMENT("inertialFilter: beta must be in [0, 2]");
        }
        if (!isValidProcessNoise(processNoise)) {
            FSW_THROW_INVALID_ARGUMENT("inertialFilter: process noise must be positive semi-definite");
        }
        if (!isValidInitialCovariance(initialCovariance)) {
            FSW_THROW_INVALID_ARGUMENT("inertialFilter: initial covariance must be positive semi-definite");
        }
        if (!isValidInitialState(initialState)) {
            FSW_THROW_INVALID_ARGUMENT("inertialFilter: initial state must be finite");
        }
        if (!isValidStMeasurementNoiseStd(stMeasurementNoiseStd)) {
            FSW_THROW_INVALID_ARGUMENT("inertialFilter: ST measurement noise std must not be negative");
        }
        if (!isValidRateMeasurementNoiseStd(rateMeasurementNoiseStd)) {
            FSW_THROW_INVALID_ARGUMENT("inertialFilter: rate measurement noise std must not be negative");
        }
        return {
            alpha, beta, processNoise, initialState, initialCovariance, stMeasurementNoiseStd, rateMeasurementNoiseStd};
    }

    static bool isValidProcessNoise(StateMatrix const& processNoise) {
        return isPositiveSemiDefinite<InertialState::size>(processNoise);
    }
    static bool isValidInitialState(InertialState const& initialState) { return initialState.allFinite(); }
    static bool isValidInitialCovariance(StateMatrix const& covariance) {
        return covariance.allFinite() && isPositiveSemiDefinite<InertialState::size>(covariance);
    }
    static bool isValidStMeasurementNoiseStd(float noiseStd) { return noiseStd >= 0.0F; }
    static bool isValidRateMeasurementNoiseStd(float noiseStd) { return noiseStd >= 0.0F; }

    float getAlpha() const { return this->alpha; }
    float getBeta() const { return this->beta; }
    StateMatrix const& getProcessNoise() const { return this->processNoise; }
    InertialState const& getInitialState() const { return this->initialState; }
    StateMatrix const& getInitialCovariance() const { return this->initialCovariance; }
    float getStMeasurementNoiseStd() const { return this->stMeasNoiseStd; }
    float getRateMeasurementNoiseStd() const { return this->rateMeasNoiseStd; }

   private:
    InertialFilterConfig(float alpha,
                         float beta,
                         StateMatrix const& processNoise,
                         InertialState const& initialState,
                         StateMatrix const& initialCovariance,
                         float stMeasurementNoiseStd,
                         float rateMeasurementNoiseStd)
        : alpha(alpha),
          beta(beta),
          processNoise(processNoise),
          initialState(initialState),
          initialCovariance(initialCovariance),
          stMeasNoiseStd(stMeasurementNoiseStd),
          rateMeasNoiseStd(rateMeasurementNoiseStd) {}

    float alpha;
    float beta;
    StateMatrix processNoise;
    InertialState initialState;
    StateMatrix initialCovariance;
    float stMeasNoiseStd;
    float rateMeasNoiseStd;
};

/*! @brief Inertial attitude square-root UKF. Estimates the inertial-to-body MRP attitude and the body
 *  angular rate from star-tracker attitude and rate measurements on one timeline. */
class InertialFilterAlgorithm {
   public:
    using State = InertialState;
    static constexpr int N = State::size;

    explicit InertialFilterAlgorithm(const InertialFilterConfig& config);

    void setConfig(InertialFilterConfig const& config);

    InertialFilterOutput update(double currentSeconds, StAttData const& stAttData, RateData const& rateData);

    void reInitializeExceptPersistentStates();
    void reInitialize();
    bool timeUpdate(float dt);
    bool measurementUpdate(Measurement const& measurement);
    void clear();

    FilterStateOutput getFilterOutput() const;
    StAttResidualsOutput const& getLastStAttResiduals() const;
    RateResidualsOutput const& getLastRateResiduals() const;
    State getState() const;
    Eigen::Matrix<float, N, N> getCovariance() const;

   private:
    bool applyMeasurement(StAttMeasurement const& measurement);
    bool applyMeasurement(RateMeasurement const& measurement);

    StAttMeasurement packStAttMeasurement(StAttData const& stAttData) const;
    RateMeasurement packRateMeasurement(RateData const& rateData) const;

    static State regularize(State const& state);

    InertialFilterConfig cfg;  //!< validated configuration, supplied at construction / setConfig()
    SRuKF<State, InertialDynamics> srukf;
    filtering::measurement_queue<Measurement, BatchSize> measurements;

    StAttResidualsOutput
        lastStAttResiduals{};  //!< latest ST residuals; valid=true only on cycles an ST measurement fired
    RateResidualsOutput
        lastRateResiduals{};  //!< latest rate residuals; valid=true only on cycles a rate measurement fired
};

}  // namespace filtering::inertialFilter

#endif
