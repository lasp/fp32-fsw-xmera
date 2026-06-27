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
    Eigen::Vector3d sigma_BN = Eigen::Vector3d::Zero();
};

/*! Raw gyro reading consumed by InertialFilterAlgorithm::update(). `timeTag` > 0
 *  flags that the adapter saw a fresh reading this cycle. */
struct RateData {
    double timeTag = 0;
    Eigen::Vector3d rate = Eigen::Vector3d::Zero();
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

    static InertialFilterConfig create(double alpha,
                                       double beta,
                                       StateMatrix const& processNoise,
                                       InertialState const& initialState,
                                       StateMatrix const& initialCovariance,
                                       double stMeasurementNoiseStd,
                                       double gyroMeasurementNoiseStd) {
        if (!Srukf::alphaIsValid(alpha)) {
            FSW_THROW_INVALID_ARGUMENT("inertialFilter: alpha must be in (0, 1)");
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
        if (!isValidStMeasurementNoiseStd(stMeasurementNoiseStd)) {
            FSW_THROW_INVALID_ARGUMENT("inertialFilter: ST measurement noise std must not be negative");
        }
        if (!isValidGyroMeasurementNoiseStd(gyroMeasurementNoiseStd)) {
            FSW_THROW_INVALID_ARGUMENT("inertialFilter: gyro measurement noise std must not be negative");
        }
        return {
            alpha, beta, processNoise, initialState, initialCovariance, stMeasurementNoiseStd, gyroMeasurementNoiseStd};
    }

    static bool isValidProcessNoise(StateMatrix const& processNoise) {
        return isPositiveSemiDefinite<InertialState::size>(processNoise);
    }
    static bool isValidInitialCovariance(StateMatrix const& covariance) {
        return isPositiveSemiDefinite<InertialState::size>(covariance);
    }
    static bool isValidStMeasurementNoiseStd(double noiseStd) { return noiseStd >= 0.0; }
    static bool isValidGyroMeasurementNoiseStd(double noiseStd) { return noiseStd >= 0.0; }

    double getAlpha() const { return this->alpha; }
    double getBeta() const { return this->beta; }
    StateMatrix const& getProcessNoise() const { return this->processNoise; }
    InertialState const& getInitialState() const { return this->initialState; }
    StateMatrix const& getInitialCovariance() const { return this->initialCovariance; }
    double getStMeasurementNoiseStd() const { return this->stMeasNoiseStd; }
    double getGyroMeasurementNoiseStd() const { return this->gyroMeasNoiseStd; }

   private:
    InertialFilterConfig(double alpha,
                         double beta,
                         StateMatrix const& processNoise,
                         InertialState const& initialState,
                         StateMatrix const& initialCovariance,
                         double stMeasurementNoiseStd,
                         double gyroMeasurementNoiseStd)
        : alpha(alpha),
          beta(beta),
          processNoise(processNoise),
          initialState(initialState),
          initialCovariance(initialCovariance),
          stMeasNoiseStd(stMeasurementNoiseStd),
          gyroMeasNoiseStd(gyroMeasurementNoiseStd) {}

    double alpha;
    double beta;
    StateMatrix processNoise;
    InertialState initialState;
    StateMatrix initialCovariance;
    double stMeasNoiseStd;
    double gyroMeasNoiseStd;
};

/*! @brief Inertial attitude square-root UKF. Estimates the inertial-to-body MRP attitude and the body
 *  angular rate from star-tracker attitude measurements and gyro rates on one timeline. */
class InertialFilterAlgorithm {
   public:
    using State = InertialState;
    static constexpr int N = State::size;

    explicit InertialFilterAlgorithm(const InertialFilterConfig& config);

    void setConfig(InertialFilterConfig const& config);

    InertialFilterOutput update(double currentSeconds, StAttData const& stAttData, RateData const& rateData);

    void reInitialize();
    void reInitializeAll();
    bool timeUpdate(double dt);
    bool measurementUpdate(Measurement const& measurement);
    void clear();

    FilterStateOutput getFilterOutput() const;
    StAttResidualsOutput const& getLastStAttResiduals() const;
    RateResidualsOutput const& getLastRateResiduals() const;
    State getState() const;
    Eigen::Matrix<double, N, N> getCovariance() const;

   private:
    bool applyMeasurement(StAttMeasurement const& measurement);
    bool applyMeasurement(RateMeasurement const& measurement);

    StAttMeasurement packStAttMeasurement(StAttData const& stAttData) const;
    RateMeasurement packRateMeasurement(RateData const& rateData) const;

    State regularize(State const& state) const;

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
