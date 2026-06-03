// SPDX-License-Identifier: ISC
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef F32XMERA_FLYBY_POINT_ALGORITHM_H
#define F32XMERA_FLYBY_POINT_ALGORITHM_H

#include <architecture/msgPayloadDef/AttRefMsgPayload.h>
#include <architecture/msgPayloadDef/FlybyDiagnosticMsgPayload.h>
#include <Eigen/Dense>

/*! structure containing the attitude guidance outputs of the algorithm */
struct FlybyPointOutput {
    Eigen::Vector3f sigma_RN = Eigen::Vector3f::Zero();
    Eigen::Vector3f omega_RN_N = Eigen::Vector3f::Zero();
    Eigen::Vector3f domega_RN_N = Eigen::Vector3f::Zero();
    bool collinearityTrigger = false;
    bool maxRateTrigger = false;
    bool maxAccelerationTrigger = false;
    bool positionKnowledgeExceedTrigger = false;
};

/*! @brief A class to perform flyby pointing */
class FlybyPointAlgorithm {
   public:
    void reset();
    FlybyPointOutput updateState(uint64_t currentSimNanos,
                                 const Eigen::Vector3d& r_BN_N,
                                 const Eigen::Vector3d& v_BN_N);
    bool checkValidity(uint64_t currentSimNanos,
                       const Eigen::Vector3d& r_BN_N,
                       const Eigen::Vector3d& v_BN_N,
                       FlybyDiagnosticMsgPayload& flybyDiagnosticMsgBuffer) const;
    void computeFlybyParameters(const Eigen::Vector3d& r_BN_N, const Eigen::Vector3d& v_BN_N);
    void computeRN(const Eigen::Vector3d& r_BN_N, const Eigen::Vector3d& v_BN_N);
    std::tuple<Eigen::Vector3f, Eigen::Vector3f, Eigen::Vector3f> computeGuidanceSolution() const;
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
    double dt = 0;                     //!< current time step between last two updates
    double timeOfFirstRead = 0;        //!< time of first nav solution read
    double timeBetweenFilterData = 0;  //!< time between two subsequent reads of the filter information
    float toleranceForCollinearity =
        0.0F;  //!< tolerance for singular conditions when position and velocity are collinear
    int signOfOrbitNormalFrameVector = 1;  //!< Sign of orbit normal vector to complete reference frame

    float maxRate = 0.0F;          //!< maximum rate spacecraft can control to, used for validity of solution
    float maxAcceleration = 0.0F;  //!< maximum acceleration spacecraft can control to, used for validity of solution

    bool firstRead = true;            //!< variable to attest if this is the first read after a Reset
    float f0 = 0.0F;                  //!< ratio between relative velocity and position norms at time of read [Hz]
    float gamma0 = 0.0F;              //!< flight path angle of the spacecraft at time of read [rad]
    uint64_t lastFilterReadTime = 0;  //!< time of last filter read
    Eigen::Matrix3f R0N{Eigen::Matrix3f::Identity()};  //!< inertial-to-reference DCM at time of read
    Eigen::Vector3d firstNavPosition{};                //!< First position used to create profile
    Eigen::Vector3d firstNavVelocity{};                //!< First velocity used to create profile
    float positionKnowledgeSigma = 0.0F;               //!< Last position used to create profile
};

#endif
