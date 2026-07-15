// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef _TWOAXISGIMBALAXISTOMOTORANGLES_
#define _TWOAXISGIMBALAXISTOMOTORANGLES_

#include <Eigen/Core>
#include <array>

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "msgPayloadDef/BodyHeadingMsgF32Payload.h"
#include "msgPayloadDef/HingedRigidBodyMsgF32Payload.h"
#include "msgPayloadDef/TwoAxisGimbalMsgF32Payload.h"
#include "twoAxisGimbalAxisToMotorAnglesAlgorithm.h"

/*! @brief Two-Axis Gimbal Axis-To-Motor Angles adapter. Reads the requested body-frame thrust
direction message, delegates the angle computation to TwoAxisGimbalAxisToMotorAnglesAlgorithm, and
writes the corresponding gimbal and stepper motor angles to the output messages. */
class TwoAxisGimbalAxisToMotorAngles : public SysModel {
   public:
    TwoAxisGimbalAxisToMotorAngles(const GimbalMotorTable& gimbalToMotor1Data,
                                   const GimbalMotorTable& gimbalToMotor2Data);  //!< Constructor
    ~TwoAxisGimbalAxisToMotorAngles() = default;                                 //!< Destructor
    void reset(uint64_t currentSimNanos) override;                               //!< Reset member function
    void updateState(uint64_t currentSimNanos) override;                         //!< Update member function
    void setDcmMB(
        const Eigen::Matrix3f& dcm_MB);       //!< Setter method for dcm_MB (DCM from body frame to gimbal mount frame)
    const Eigen::Matrix3f& getDcmMB() const;  //!< Getter method for dcm_MB (DCM from body frame to gimbal mount frame)

    ReadFunctor<BodyHeadingMsgF32Payload>
        thrustDirectionInMsg;  //!< Input msg for the requested gimbal body-frame thrust direction vector
    Message<TwoAxisGimbalMsgF32Payload>
        twoAxisGimbalOutMsg;  //!< Output msg for the corresponding gimbal tip and tilt angles
    Message<HingedRigidBodyMsgF32Payload> motor1AngleOutMsg;  //!< Output message for the motor 1 angle
    Message<HingedRigidBodyMsgF32Payload> motor2AngleOutMsg;  //!< Output message for the motor 2 angle

   private:
    void rebuildAlgorithmConfig();  //!< Rebuild the algorithm's Config from the stored parameters

    double previousWrittenTime{-1.0};                      //!< [s] Time the previous input message was written
    Eigen::Matrix3f dcm_MB = Eigen::Matrix3f::Identity();  //!< DCM from body frame to gimbal mount frame
    GimbalMotorTable gimbalToMotor1Data{};                 //!< [rad] Gimbal-to-motor 1 angle interpolation table
    GimbalMotorTable gimbalToMotor2Data{};                 //!< [rad] Gimbal-to-motor 2 angle interpolation table
    TwoAxisGimbalAxisToMotorAnglesAlgorithm algorithm{
        TwoAxisGimbalAxisToMotorAnglesConfig::create(Eigen::Matrix3f::Identity(),
                                                     GimbalMotorTable{},
                                                     GimbalMotorTable{})};  //!< Angle computation algorithm
};

#endif /* TWOAXISGIMBALAXISTOMOTORANGLES */
