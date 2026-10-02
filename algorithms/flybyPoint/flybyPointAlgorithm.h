#ifndef F32XMERA_FLYBY_POINT_ALGORITHM_H
#define F32XMERA_FLYBY_POINT_ALGORITHM_H

#include "flybyPointTypes.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/freestandingIsFinite.hpp"
#include <Eigen/Dense>
#include <cstdint>
#include <optional>

/*! @brief Structure containing the attitude guidance output of the algorithm. As in the other guidance algorithms,
 * the reference (sigma_RN, omega_RN_N, domega_RN_N) is all zero when no solution is available: before the first seed,
 * or if the guidance solution is not finite. */
struct AttGuideOutput {
    Eigen::Vector3f sigma_RN = Eigen::Vector3f::Zero();
    Eigen::Vector3f omega_RN_N = Eigen::Vector3f::Zero();
    Eigen::Vector3f domega_RN_N = Eigen::Vector3f::Zero();
    bool collinearityTrigger = false;     // true if vectors r and v are collinear
    bool maxRateTrigger = false;          // true if the predicted rate exceeds the maximum rate of the spacecraft
    bool maxAccelerationTrigger = false;  // true if the predicted acceleration exceeds the maximum acceleration of the
                                          // spacecraft
    bool positionKnowledgeExceedTrigger = false;  // true if the position error exceeds a-priori sigma bound
    bool inputSampleRejected = false;  // true if this period's filter sample was unusable and left out of the average
    uint32_t rejectedSamplesInWindow = 0;  // number of unusable samples in the window ending this period, else 0
};

/*! @brief Which validity checks rejected a re-read candidate; all false when none did. */
struct FlybyValidityTriggers {
    bool collinearityTrigger = false;             // true if vectors r and v are collinear
    bool maxRateTrigger = false;                  // true if the predicted peak rate exceeds the maximum rate
    bool maxAccelerationTrigger = false;          // true if the predicted peak acceleration exceeds the maximum
    bool positionKnowledgeExceedTrigger = false;  // true if the position error exceeds a-priori sigma bound
};

/*!
 * @brief Validated configuration for flyby pointing. Construct via FlybyPointConfig::create(...).
 *
 * The filter re-read cadence is a whole number of control periods, so the decision of when to re-read is exact
 * integer arithmetic. Elapsed time in seconds is derived as (control periods elapsed) * controlPeriod.
 */
class FlybyPointConfig final {
   public:
    static FlybyPointConfig create(double controlPeriod,
                                   uint32_t filterReadPeriods,
                                   float toleranceForCollinearity,
                                   int signOfOrbitNormalFrameVector,
                                   float maximumRateThreshold,
                                   float maximumAccelerationThreshold,
                                   float positionKnowledgeSigma) {
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
        if (!isValidPositionKnowledgeSigma(positionKnowledgeSigma)) {
            FSW_THROW_INVALID_ARGUMENT("flybyPoint: positionKnowledgeSigma must be > 0");
        }
        return {controlPeriod,
                filterReadPeriods,
                toleranceForCollinearity,
                signOfOrbitNormalFrameVector,
                maximumRateThreshold,
                maximumAccelerationThreshold,
                positionKnowledgeSigma};
    }

    static bool isValidControlPeriod(double t) { return fsw::is_finite(t) && t > 0.0; }
    /*! The filter is re-read at most once per control period, so the shortest cadence is one period. */
    static bool isValidFilterReadPeriods(uint32_t n) { return n >= 1U; }
    static bool isValidToleranceForCollinearity(float t) { return t > 0.0F; }
    static bool isValidSignOfOrbitNormalFrameVector(int s) { return s == 1 || s == -1; }
    static bool isValidMaximumRateThreshold(float r) { return r > 0.0F; }
    static bool isValidMaximumAccelerationThreshold(float a) { return a > 0.0F; }
    static bool isValidPositionKnowledgeSigma(float s) { return s > 0.0F; }

    /*! @return [s] time between successive updateState() calls */
    double getControlPeriod() const { return controlPeriod; }
    /*! @return [-] control periods between two consecutive filter re-reads */
    uint32_t getFilterReadPeriods() const { return filterReadPeriods; }
    float getToleranceForCollinearity() const { return toleranceForCollinearity; }
    int getSignOfOrbitNormalFrameVector() const { return signOfOrbitNormalFrameVector; }
    float getMaximumRateThreshold() const { return maximumRateThreshold; }
    float getMaximumAccelerationThreshold() const { return maximumAccelerationThreshold; }
    float getPositionKnowledgeSigma() const { return positionKnowledgeSigma; }

   private:
    FlybyPointConfig(double controlPeriod,  // NOLINT(bugprone-easily-swappable-parameters)
                     uint32_t filterReadPeriods,
                     float toleranceForCollinearity,
                     int signOfOrbitNormalFrameVector,
                     float maximumRateThreshold,
                     float maximumAccelerationThreshold,
                     float positionKnowledgeSigma)
        : controlPeriod(controlPeriod),
          filterReadPeriods(filterReadPeriods),
          toleranceForCollinearity(toleranceForCollinearity),
          signOfOrbitNormalFrameVector(signOfOrbitNormalFrameVector),
          maximumRateThreshold(maximumRateThreshold),
          maximumAccelerationThreshold(maximumAccelerationThreshold),
          positionKnowledgeSigma(positionKnowledgeSigma) {}

    double controlPeriod;        //!< [s] time between successive updateState() calls
    uint32_t filterReadPeriods;  //!< [-] control periods between two consecutive filter re-reads
    float toleranceForCollinearity;
    int signOfOrbitNormalFrameVector;
    float maximumRateThreshold;
    float maximumAccelerationThreshold;
    float positionKnowledgeSigma;
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
 * An unusable sample is left out of the average and reported through the diagnostics; it does not affect the
 * guidance output, which comes from the last accepted profile. The output is therefore a valid reference on every
 * period after the first seed. The first seed must not be collinear, since r and v then define no orbit normal.
 */
class FlybyPointAlgorithm final {
   public:
    /*! [-] smallest |r_hat x v_hat| (sine of the r-v angle) for which r and v define an orbit normal. Below it the
     orbit-normal direction would be dominated by rounding, so the pair is treated as collinear whatever
     toleranceForCollinearity is; this only matters for a toleranceForCollinearity below double resolution. */
    static constexpr double kMinOrbitNormalNorm = 1e-12;

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

    /*! Pointing profile from the last accepted read; the position-knowledge check compares re-reads with the read's
     rectilinear prediction */
    struct Profile {
        Eigen::Vector3d r_N = Eigen::Vector3d::Zero();      //!< [m] position of the accepted read
        Eigen::Vector3d v_N = Eigen::Vector3d::Zero();      //!< [m/s] velocity of the accepted read
        double f0 = 0;                                      //!< [1/s] |v| / |r| at the read
        double gamma0 = 0;                                  //!< [rad] flight path angle at the read
        Eigen::Matrix3f R0N = Eigen::Matrix3f::Identity();  //!< [-] inertial-to-reference DCM at the read
        uint64_t periodsSinceRead = 0;                      //!< [-] control periods elapsed since the read
    };

    void seedProfile(const Eigen::Vector3d& r_BN_N, const Eigen::Vector3d& v_BN_N);
    FlybyValidityTriggers reReadFromWindow(const Profile& p);
    GuidanceReference computeGuidanceReference(const Profile& p) const;

    FlybyPointConfig cfg;
    std::optional<Profile> profile;  //!< empty until the first seed, and again after reset()
    AveragingWindow window{};        //!< averaging window in progress
};

#endif  // F32XMERA_FLYBY_POINT_ALGORITHM_H
