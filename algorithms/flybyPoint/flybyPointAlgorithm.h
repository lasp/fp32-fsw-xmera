#ifndef F32XMERA_FLYBY_POINT_ALGORITHM_H
#define F32XMERA_FLYBY_POINT_ALGORITHM_H

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
    double getToleranceForCollinearity() const;
    void setToleranceForCollinearity(double toleranceForCollinearity);
    int getSignOfOrbitNormalFrameVector() const;
    void setSignOfOrbitNormalFrameVector(int signOfOrbitNormalFrameVector);
    double getMaximumAccelerationThreshold() const;
    void setMaximumAccelerationThreshold(double maxAccelerationThreshold);
    double getMaximumRateThreshold() const;
    void setMaximumRateThreshold(double maxRateThreshold);
    double getPositionKnowledgeSigma() const;
    void setPositionKnowledgeSigma(double positionKnowledgeStd);

   private:
    double dt = 0;                     //!< current time step between last two updates
    double timeOfFirstRead = 0;        //!< time of first nav solution read
    double timeBetweenFilterData = 0;  //!< time between two subsequent reads of the filter information
    double toleranceForCollinearity =
        0;  //!< tolerance for singular conditions when position and velocity are collinear
    int signOfOrbitNormalFrameVector = 1;  //!< Sign of orbit normal vector to complete reference frame

    double maxRate = 0;          //!< maximum rate spacecraft can control to, used for validity of solution
    double maxAcceleration = 0;  //!< maximum acceleration spacecraft can control to, used for validity of solution

    bool firstRead = true;            //!< variable to attest if this is the first read after a Reset
    double f0 = 0;                    //!< ratio between relative velocity and position norms at time of read [Hz]
    double gamma0 = 0;                //!< flight path angle of the spacecraft at time of read [rad]
    uint64_t lastFilterReadTime = 0;  //!< time of last filter read
    Eigen::Matrix3d R0N{Eigen::Matrix3d::Identity()};            //!< inertial-to-reference DCM at time of read
    Eigen::Vector3d firstNavPosition = Eigen::Vector3d::Zero();  //!< First position used to create profile
    Eigen::Vector3d firstNavVelocity = Eigen::Vector3d::Zero();  //!< First velocity used to create profile
    double positionKnowledgeSigma = 0;                           //!< Last position used to create profile
};

#endif  // F32XMERA_FLYBY_POINT_ALGORITHM_H
