#ifndef F32XMERA_FLYBY_POINT_ALGORITHM_H
#define F32XMERA_FLYBY_POINT_ALGORITHM_H

#include "flybyPointTypes.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/freestandingIsFinite.hpp"
#include <Eigen/Dense>
#include <cstdint>

/*! @brief Structure containing the attitude guidance output of the algorithm */
struct AttGuideOutput {
    Eigen::Vector3f sigma_RN = Eigen::Vector3f::Zero();
    Eigen::Vector3f omega_RN_N = Eigen::Vector3f::Zero();
    Eigen::Vector3f domega_RN_N = Eigen::Vector3f::Zero();
    bool collinearityTrigger = false;     // true if vectors r and v are collinear
    bool maxRateTrigger = false;          // true if the predicted rate exceeds the maximum rate of the spacecraft
    bool maxAccelerationTrigger = false;  // true if the predicted acceleration exceeds the maximum acceleration of the
                                          // spacecraft
    bool positionKnowledgeExceedTrigger = false;  // true if the position error exceeds a-priori sigma bound
    bool validOutput = false;
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
 */
class FlybyPointAlgorithm final {
   public:
    explicit FlybyPointAlgorithm(const FlybyPointConfig& config);
    void setConfig(const FlybyPointConfig& config);
    void reset();
    AttGuideOutput updateState(const Eigen::Vector3d& r_BN_N, const Eigen::Vector3d& v_BN_N);

   private:
    bool checkValidity(const Eigen::Vector3d& r_BN_N, const Eigen::Vector3d& v_BN_N, AttGuideOutput& output) const;
    void computeFlybyParameters(const Eigen::Vector3d& r_BN_N, const Eigen::Vector3d& v_BN_N);
    void computeRN(const Eigen::Vector3d& r_BN_N, const Eigen::Vector3d& v_BN_N);
    std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d> computeGuidanceSolution(double dt) const;
    FlybyPointConfig cfg;
    bool firstRead = true;               //!< variable to attest if this is the first read after a Reset
    uint64_t periodsSinceLastRead = 0;   //!< [-] control periods elapsed since the last accepted filter read
    uint64_t periodsSinceFirstRead = 0;  //!< [-] control periods elapsed since the first filter read
    double f0 = 0;                       //!< ratio between relative velocity and position norms at time of read [Hz]
    double gamma0 = 0;                   //!< flight path angle of the spacecraft at time of read [rad]
    Eigen::Matrix3f R0N{Eigen::Matrix3f::Identity()};            //!< inertial-to-reference DCM at time of read
    Eigen::Vector3d firstNavPosition = Eigen::Vector3d::Zero();  //!< First position used to create profile
    Eigen::Vector3d firstNavVelocity = Eigen::Vector3d::Zero();  //!< First velocity used to create profile
};

#endif  // F32XMERA_FLYBY_POINT_ALGORITHM_H
