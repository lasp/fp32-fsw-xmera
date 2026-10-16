#ifndef F32XMERA_FLYBY_POINT_ALGORITHM_H
#define F32XMERA_FLYBY_POINT_ALGORITHM_H

#include "flybyPointTypes.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/freestandingIsFinite.hpp"
#include <math.h>
#include <Eigen/Dense>
#include <cstdint>

/*! @brief Structure containing the attitude guidance output of the algorithm. As in the other guidance algorithms,
 * the reference (sigma_RN, omega_RN_N, domega_RN_N) is all zero when no solution is available, which happens only if
 * the guidance solution is not finite. */
struct AttGuideOutput {
    Eigen::Vector3f sigma_RN = Eigen::Vector3f::Zero();
    Eigen::Vector3f omega_RN_N = Eigen::Vector3f::Zero();
    Eigen::Vector3f domega_RN_N = Eigen::Vector3f::Zero();
    bool collinearityTrigger = false;     // true if vectors r and v are collinear
    bool maxRateTrigger = false;          // true if the predicted rate exceeds the maximum rate of the spacecraft
    bool maxAccelerationTrigger = false;  // true if the predicted acceleration exceeds the maximum acceleration of the
                                          // spacecraft
    bool stateKnowledgeExceedTrigger = false;  // true if the position or velocity error exceeds the propagated ground
                                               // sigma bound
    bool inputSampleRejected = false;  // true if this period's filter sample was unusable and left out of the average
    uint32_t rejectedSamplesInWindow = 0;  // number of unusable samples in the window ending this period, else 0
};

/*! @brief Which validity checks rejected a re-read candidate; all false when none did. */
struct FlybyValidityTriggers {
    bool collinearityTrigger = false;          // true if vectors r and v are collinear
    bool maxRateTrigger = false;               // true if the predicted peak rate exceeds the maximum rate
    bool maxAccelerationTrigger = false;       // true if the predicted peak acceleration exceeds the maximum
    bool stateKnowledgeExceedTrigger = false;  // true if the position or velocity error exceeds the propagated ground
                                               // sigma bound
};

/*!
 * @brief Validated configuration for flyby pointing. Construct via FlybyPointConfig::create(...).
 *
 * The filter re-read cadence is a whole number of control periods, so the decision of when to re-read is exact
 * integer arithmetic. Elapsed time in seconds is derived as (control periods elapsed) * controlPeriod.
 *
 * The ground-based position and velocity seed the pointing profile when the algorithm is configured or reset. They are
 * the relative state at the time of the first updateState() call that follows, and must be a usable, non-collinear
 * pair, since r and v must define an orbit normal. The ground-based state and the diagonal terms (variances) of its
 * covariance, both propagated from that time, give the trajectory each re-read is checked against and the per-axis
 * bound on its deviation, scaled by numberOfSigmas.
 */
class FlybyPointConfig final {
   public:
    /*! [-] smallest |r_hat x v_hat| (sine of the r-v angle) for which r and v define an orbit normal. Below it the
     orbit-normal direction would be dominated by rounding, so the pair is treated as collinear whatever
     toleranceForCollinearity is; this only matters for a toleranceForCollinearity below double resolution. */
    static constexpr double kMinOrbitNormalNorm = 1e-12;
    /*! [m] smallest |r| of a usable state. Below it r_hat and f0 = |v| / |r| are dominated by rounding, so the state
     can't define a profile. Far below any real flyby distance */
    static constexpr double kMinPositionNorm = 1e-3;
    /*! [m/s] smallest |v| of a usable state. Below it v_hat, and so the orbit normal, is dominated by rounding. Far
     below any real flyby speed  */
    static constexpr double kMinVelocityNorm = 1e-3;

    static FlybyPointConfig create(double controlPeriod,
                                   uint32_t filterReadPeriods,
                                   float toleranceForCollinearity,
                                   int signOfOrbitNormalFrameVector,
                                   float maximumRateThreshold,
                                   float maximumAccelerationThreshold,
                                   float numberOfSigmas,
                                   const Eigen::Vector3d& groundPosition_BN_N,
                                   const Eigen::Vector3d& groundVelocity_BN_N,
                                   const Eigen::Vector3d& groundPositionCovarianceDiag_N,
                                   const Eigen::Vector3d& groundVelocityCovarianceDiag_N) {
        if (!isValidControlPeriod(controlPeriod)) {
            FSW_THROW_INVALID_ARGUMENT("flybyPoint: controlPeriod must be finite and > 0");
        }
        if (!isValidFilterReadPeriods(filterReadPeriods)) {
            FSW_THROW_INVALID_ARGUMENT("flybyPoint: filterReadPeriods must be >= 1");
        }
        if (!isValidToleranceForCollinearity(toleranceForCollinearity)) {
            FSW_THROW_INVALID_ARGUMENT("flybyPoint: toleranceForCollinearity must be > 0");
        }
        if (!isValidSignOfOrbitNormalFrameVector(signOfOrbitNormalFrameVector)) {
            FSW_THROW_INVALID_ARGUMENT("flybyPoint: signOfOrbitNormalFrameVector must be +1 or -1");
        }
        if (!isValidMaximumRateThreshold(maximumRateThreshold)) {
            FSW_THROW_INVALID_ARGUMENT("flybyPoint: maximumRateThreshold must be > 0");
        }
        if (!isValidMaximumAccelerationThreshold(maximumAccelerationThreshold)) {
            FSW_THROW_INVALID_ARGUMENT("flybyPoint: maximumAccelerationThreshold must be > 0");
        }
        if (!isValidNumberOfSigmas(numberOfSigmas)) {
            FSW_THROW_INVALID_ARGUMENT("flybyPoint: numberOfSigmas must be finite and > 0");
        }
        if (!isUsableSample(groundPosition_BN_N, groundVelocity_BN_N)) {
            FSW_THROW_INVALID_ARGUMENT(
                "flybyPoint: groundPosition_BN_N and groundVelocity_BN_N must be finite, non-zero and not overflow");
        }
        if (isCollinear(groundPosition_BN_N, groundVelocity_BN_N, toleranceForCollinearity)) {
            FSW_THROW_INVALID_ARGUMENT("flybyPoint: groundPosition_BN_N and groundVelocity_BN_N must not be collinear");
        }
        if (!isValidGroundPositionCovarianceDiag(groundPositionCovarianceDiag_N)) {
            FSW_THROW_INVALID_ARGUMENT("flybyPoint: groundPositionCovarianceDiag_N must be finite and > 0");
        }
        if (!isValidGroundVelocityCovarianceDiag(groundVelocityCovarianceDiag_N)) {
            FSW_THROW_INVALID_ARGUMENT("flybyPoint: groundVelocityCovarianceDiag_N must be finite and > 0");
        }
        return {controlPeriod,
                filterReadPeriods,
                toleranceForCollinearity,
                signOfOrbitNormalFrameVector,
                maximumRateThreshold,
                maximumAccelerationThreshold,
                numberOfSigmas,
                groundPosition_BN_N,
                groundVelocity_BN_N,
                groundPositionCovarianceDiag_N,
                groundVelocityCovarianceDiag_N};
    }

    static bool isValidControlPeriod(double t) { return fsw::is_finite(t) && t > 0.0; }
    /*! The filter is re-read at most once per control period, so the shortest cadence is one period. */
    static bool isValidFilterReadPeriods(uint32_t n) { return n >= 1U; }
    static bool isValidToleranceForCollinearity(float t) { return t > 0.0F; }
    static bool isValidSignOfOrbitNormalFrameVector(int s) { return s == 1 || s == -1; }
    static bool isValidMaximumRateThreshold(float r) { return r > 0.0F; }
    static bool isValidMaximumAccelerationThreshold(float a) { return a > 0.0F; }
    static bool isValidNumberOfSigmas(float n) { return fsw::is_finite(n) && n > 0.0F; }
    /*! Check whether the ground state can seed the profile: it must be a usable, non-collinear sample. */
    static bool isValidGroundState(const Eigen::Vector3d& r_BN_N,
                                   const Eigen::Vector3d& v_BN_N,
                                   float toleranceForCollinearity) {
        return isUsableSample(r_BN_N, v_BN_N) && !isCollinear(r_BN_N, v_BN_N, toleranceForCollinearity);
    }
    /*! A diagonal covariance is positive definite if and only if every diagonal term is > 0; a zero variance would
     reject any deviation on that axis. */
    static bool isValidGroundPositionCovarianceDiag(const Eigen::Vector3d& p) {
        return p.allFinite() && (p.array() > 0.0).all();
    }
    static bool isValidGroundVelocityCovarianceDiag(const Eigen::Vector3d& p) {
        return p.allFinite() && (p.array() > 0.0).all();
    }

    /*! Check whether a state can seed the profile or enter the averaging window: it must be finite, with
     |r| >= kMinPositionNorm and |v| >= kMinVelocityNorm, and the products formed from it must be finite, |r||v|
     (closest approach) and f0^2 = (|v|/|r|)^2 (frame acceleration). Used by create() for the ground state, and by
     updateState() for each filter sample and for the window average.
     @return true if the sample is usable
     @param r_BN_N [m] relative position
     @param v_BN_N [m/s] relative velocity
     */
    static bool isUsableSample(const Eigen::Vector3d& r_BN_N, const Eigen::Vector3d& v_BN_N) {
        const double rNorm = r_BN_N.stableNorm();
        const double vNorm = v_BN_N.stableNorm();
        return r_BN_N.allFinite() && v_BN_N.allFinite() && rNorm >= kMinPositionNorm && vNorm >= kMinVelocityNorm &&
               fsw::is_finite(rNorm * vNorm) && fsw::is_finite((vNorm / rNorm) * (vNorm / rNorm));
    }

    /*! Check whether r and v are collinear: 1 - |cos| of their angle is below toleranceForCollinearity, or
     |r_hat x v_hat| is below kMinOrbitNormalNorm.
     @return true if r and v are collinear
     @param r_BN_N [m] relative position
     @param v_BN_N [m/s] relative velocity
     @param toleranceForCollinearity [-] tolerance on 1 - |cos| of the r-v angle
     */
    static bool isCollinear(const Eigen::Vector3d& r_BN_N,
                            const Eigen::Vector3d& v_BN_N,
                            const float toleranceForCollinearity) {
        const Eigen::Vector3d ur_N = r_BN_N.stableNormalized();
        const Eigen::Vector3d uv_N = v_BN_N.stableNormalized();
        return 1.0 - fabs(ur_N.dot(uv_N)) < toleranceForCollinearity ||
               ur_N.cross(uv_N).stableNorm() < kMinOrbitNormalNorm;
    }

    /*! @return [s] time between successive updateState() calls */
    double getControlPeriod() const { return controlPeriod; }
    /*! @return [-] control periods between two consecutive filter re-reads */
    uint32_t getFilterReadPeriods() const { return filterReadPeriods; }
    float getToleranceForCollinearity() const { return toleranceForCollinearity; }
    int getSignOfOrbitNormalFrameVector() const { return signOfOrbitNormalFrameVector; }
    float getMaximumRateThreshold() const { return maximumRateThreshold; }
    float getMaximumAccelerationThreshold() const { return maximumAccelerationThreshold; }
    /*! @return [-] number of propagated ground sigmas a re-read may deviate from the ground-based trajectory */
    float getNumberOfSigmas() const { return numberOfSigmas; }
    /*! @return [m] ground-based position at the first updateState() call after configuring or resetting */
    const Eigen::Vector3d& getGroundPosition_BN_N() const { return groundPosition_BN_N; }
    /*! @return [m/s] ground-based velocity at the first updateState() call after configuring or resetting */
    const Eigen::Vector3d& getGroundVelocity_BN_N() const { return groundVelocity_BN_N; }
    /*! @return [m^2] diagonal terms of the ground-based position covariance */
    const Eigen::Vector3d& getGroundPositionCovarianceDiag_N() const { return groundPositionCovarianceDiag_N; }
    /*! @return [(m/s)^2] diagonal terms of the ground-based velocity covariance */
    const Eigen::Vector3d& getGroundVelocityCovarianceDiag_N() const { return groundVelocityCovarianceDiag_N; }

   private:
    FlybyPointConfig(double controlPeriod,  // NOLINT(bugprone-easily-swappable-parameters)
                     uint32_t filterReadPeriods,
                     float toleranceForCollinearity,
                     int signOfOrbitNormalFrameVector,
                     float maximumRateThreshold,
                     float maximumAccelerationThreshold,
                     float numberOfSigmas,
                     const Eigen::Vector3d& groundPosition_BN_N,
                     const Eigen::Vector3d& groundVelocity_BN_N,
                     const Eigen::Vector3d& groundPositionCovarianceDiag_N,
                     const Eigen::Vector3d& groundVelocityCovarianceDiag_N)
        : controlPeriod(controlPeriod),
          filterReadPeriods(filterReadPeriods),
          toleranceForCollinearity(toleranceForCollinearity),
          signOfOrbitNormalFrameVector(signOfOrbitNormalFrameVector),
          maximumRateThreshold(maximumRateThreshold),
          maximumAccelerationThreshold(maximumAccelerationThreshold),
          numberOfSigmas(numberOfSigmas),
          groundPosition_BN_N(groundPosition_BN_N),
          groundVelocity_BN_N(groundVelocity_BN_N),
          groundPositionCovarianceDiag_N(groundPositionCovarianceDiag_N),
          groundVelocityCovarianceDiag_N(groundVelocityCovarianceDiag_N) {}

    double controlPeriod;        //!< [s] time between successive updateState() calls
    uint32_t filterReadPeriods;  //!< [-] control periods between two consecutive filter re-reads
    float toleranceForCollinearity;
    int signOfOrbitNormalFrameVector;
    float maximumRateThreshold;
    float maximumAccelerationThreshold;
    float numberOfSigmas;  //!< [-] number of propagated ground sigmas allowed for the state knowledge check
    Eigen::Vector3d groundPosition_BN_N;             //!< [m] ground-based position
    Eigen::Vector3d groundVelocity_BN_N;             //!< [m/s] ground-based velocity
    Eigen::Vector3d groundPositionCovarianceDiag_N;  //!< [m^2] diagonal of the ground-based position covariance
    Eigen::Vector3d groundVelocityCovarianceDiag_N;  //!< [(m/s)^2] diagonal of the ground-based velocity covariance
};

/*! @brief A class to perform flyby pointing
 *
 * The algorithm has no time input: the caller must call updateState() once per control period. Time is counted
 * in whole control periods, and converted to seconds only where the continuous guidance equations need it.
 *
 * The filter states are low-pass filtered by batch averaging. Every usable sample in a window of filterReadPeriods
 * control periods is propagated to the window end with the rectilinear model and accumulated. At the window end the
 * average is the re-read candidate. With filterReadPeriods = 1 the average is the current sample, so the algorithm
 * re-reads a single sample, as without averaging.
 *
 * The profile is seeded from the configuration's ground-based state when the algorithm is constructed, reconfigured
 * or reset, so a profile always exists. The ground state is the state at the next updateState() call, which outputs
 * the profile at zero elapsed time and opens the first averaging window.
 *
 * An unusable sample is left out of the average and reported through the diagnostics; it does not affect the
 * guidance output, which comes from the last accepted profile. The output is therefore a valid reference on every
 * period.
 */
class FlybyPointAlgorithm final {
   public:
    explicit FlybyPointAlgorithm(const FlybyPointConfig& config);
    void setConfig(const FlybyPointConfig& config);
    void reset();
    AttGuideOutput updateState(const Eigen::Vector3d& r_BN_N, const Eigen::Vector3d& v_BN_N);

   private:
    /*! Running sums of the current averaging window */
    struct AveragingWindow {
        uint32_t periods = 0;                                   //!< [-] control periods elapsed in the window
        uint32_t samples = 0;                                   //!< [-] usable samples accumulated
        Eigen::Vector3d rSumAtEnd_N = Eigen::Vector3d::Zero();  //!< [m] sum of positions propagated to the window end
        Eigen::Vector3d vSum_N = Eigen::Vector3d::Zero();       //!< [m/s] sum of velocities
    };

    /*! Reference attitude, rate and acceleration of the propagated profile; all zero when it has no finite solution */
    struct GuidanceReference {
        Eigen::Vector3f sigma_RN = Eigen::Vector3f::Zero();     //!< [-] reference attitude MRP
        Eigen::Vector3f omega_RN_N = Eigen::Vector3f::Zero();   //!< [rad/s] reference angular rate
        Eigen::Vector3f domega_RN_N = Eigen::Vector3f::Zero();  //!< [rad/s^2] reference angular acceleration
    };

    /*! Pointing profile from the last accepted read, or from the ground state */
    struct Profile {
        double f0 = 0;                                      //!< [1/s] |v| / |r| at the read
        double gamma0 = 0;                                  //!< [rad] flight path angle at the read
        Eigen::Matrix3f R0N = Eigen::Matrix3f::Identity();  //!< [-] inertial-to-reference DCM at the read
        uint64_t periodsSinceRead = 0;                      //!< [-] control periods elapsed since the read
    };

    static Profile makeProfile(const Eigen::Vector3d& r_BN_N, const Eigen::Vector3d& v_BN_N);
    FlybyValidityTriggers reReadFromWindow();
    GuidanceReference computeGuidanceReference(const Profile& p) const;

    FlybyPointConfig cfg;
    Profile profile;                       //!< profile of the last accepted read, or of the ground state
    AveragingWindow window{};              //!< averaging window in progress
    uint64_t periodsSinceGroundState = 0;  //!< [-] control periods elapsed since the ground state (t0)
};

#endif  // F32XMERA_FLYBY_POINT_ALGORITHM_H
