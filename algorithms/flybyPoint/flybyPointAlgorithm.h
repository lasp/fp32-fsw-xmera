#ifndef F32XMERA_FLYBY_POINT_ALGORITHM_H
#define F32XMERA_FLYBY_POINT_ALGORITHM_H

#include "utilities/fsw/freestandingInvalidArgument.h"
#include <Eigen/Dense>

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
};

class FlybyPointConfig final {
   public:
    static FlybyPointConfig create(double timeBetweenFilterData,
                                   float toleranceForCollinearity,
                                   int signOfOrbitNormalFrameVector,
                                   float maximumRateThreshold,
                                   float maximumAccelerationThreshold,
                                   float positionKnowledgeSigma) {
        if (!isValidTimeBetweenFilterData(timeBetweenFilterData)) {
            FSW_THROW_INVALID_ARGUMENT("flybyPoint: timeBetweenFilterData must be > 0");
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
        return {timeBetweenFilterData,
                toleranceForCollinearity,
                signOfOrbitNormalFrameVector,
                maximumRateThreshold,
                maximumAccelerationThreshold,
                positionKnowledgeSigma};
    }

    static bool isValidTimeBetweenFilterData(double t) { return t > 0.0; }
    static bool isValidToleranceForCollinearity(float t) { return t > 0.0F; }
    static bool isValidSignOfOrbitNormalFrameVector(int s) { return s == 1 || s == -1; }
    static bool isValidMaximumRateThreshold(float r) { return r > 0.0F; }
    static bool isValidMaximumAccelerationThreshold(float a) { return a > 0.0F; }
    static bool isValidPositionKnowledgeSigma(float s) { return s > 0.0F; }

    double getTimeBetweenFilterData() const { return timeBetweenFilterData; }
    float getToleranceForCollinearity() const { return toleranceForCollinearity; }
    int getSignOfOrbitNormalFrameVector() const { return signOfOrbitNormalFrameVector; }
    float getMaximumRateThreshold() const { return maximumRateThreshold; }
    float getMaximumAccelerationThreshold() const { return maximumAccelerationThreshold; }
    float getPositionKnowledgeSigma() const { return positionKnowledgeSigma; }

   private:
    FlybyPointConfig(double timeBetweenFilterData,
                     float toleranceForCollinearity,
                     int signOfOrbitNormalFrameVector,
                     float maximumRateThreshold,
                     float maximumAccelerationThreshold,
                     float positionKnowledgeSigma)
        : timeBetweenFilterData(timeBetweenFilterData),
          toleranceForCollinearity(toleranceForCollinearity),
          signOfOrbitNormalFrameVector(signOfOrbitNormalFrameVector),
          maximumRateThreshold(maximumRateThreshold),
          maximumAccelerationThreshold(maximumAccelerationThreshold),
          positionKnowledgeSigma(positionKnowledgeSigma) {}

    double timeBetweenFilterData;
    float toleranceForCollinearity;
    int signOfOrbitNormalFrameVector;
    float maximumRateThreshold;
    float maximumAccelerationThreshold;
    float positionKnowledgeSigma;
};

/*! @brief A class to perform flyby pointing */
class FlybyPointAlgorithm final {
   public:
    void reset();
    AttGuideOutput updateState(uint64_t currentSimNanos, const Eigen::Vector3d& r_BN_N, const Eigen::Vector3d& v_BN_N);
    bool checkValidity(uint64_t currentSimNanos,
                       const Eigen::Vector3d& r_BN_N,
                       const Eigen::Vector3d& v_BN_N,
                       AttGuideOutput& output) const;
    void computeFlybyParameters(const Eigen::Vector3d& r_BN_N, const Eigen::Vector3d& v_BN_N);
    void computeRN(const Eigen::Vector3d& r_BN_N, const Eigen::Vector3d& v_BN_N);
    std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d> computeGuidanceSolution() const;
    double getTimeBetweenFilterData() const;
    void setTimeBetweenFilterData(double timeBetweenFilterData);
    float getToleranceForCollinearity() const;
    void setToleranceForCollinearity(float toleranceForCollinearity);
    int getSignOfOrbitNormalFrameVector() const;
    void setSignOfOrbitNormalFrameVector(int signOfOrbitNormalFrameVector);
    float getMaximumAccelerationThreshold() const;
    void setMaximumAccelerationThreshold(float maxAccelerationThreshold);
    float getMaximumRateThreshold() const;
    void setMaximumRateThreshold(float maxRateThreshold);
    float getPositionKnowledgeSigma() const;
    void setPositionKnowledgeSigma(float positionKnowledgeStd);

   private:
    double dt = 0;                       //!< current time step between last two updates
    double timeOfFirstRead = 0;          //!< time of first nav solution read
    double timeBetweenFilterData = 0;    //!< time between two subsequent reads of the filter information
    float toleranceForCollinearity = 0;  //!< tolerance for singular conditions when position and velocity are collinear
    int signOfOrbitNormalFrameVector = 1;  //!< Sign of orbit normal vector to complete reference frame

    float maxRate = 0;          //!< maximum rate spacecraft can control to, used for validity of solution
    float maxAcceleration = 0;  //!< maximum acceleration spacecraft can control to, used for validity of solution

    bool firstRead = true;            //!< variable to attest if this is the first read after a Reset
    double f0 = 0;                    //!< ratio between relative velocity and position norms at time of read [Hz]
    double gamma0 = 0;                //!< flight path angle of the spacecraft at time of read [rad]
    uint64_t lastFilterReadTime = 0;  //!< time of last filter read
    Eigen::Matrix3f R0N{Eigen::Matrix3f::Identity()};            //!< inertial-to-reference DCM at time of read
    Eigen::Vector3d firstNavPosition = Eigen::Vector3d::Zero();  //!< First position used to create profile
    Eigen::Vector3d firstNavVelocity = Eigen::Vector3d::Zero();  //!< First velocity used to create profile
    float positionKnowledgeSigma = 0;                            //!< Last position used to create profile
};

#endif  // F32XMERA_FLYBY_POINT_ALGORITHM_H
