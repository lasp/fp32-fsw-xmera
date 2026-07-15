// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef _TWOAXISGIMBALAXISTOMOTORANGLES_
#define _TWOAXISGIMBALAXISTOMOTORANGLES_

#include <Eigen/Core>

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDef/BodyHeadingMsgPayload.h"
#include "architecture/msgPayloadDef/HingedRigidBodyMsgPayload.h"
#include "architecture/msgPayloadDef/TwoAxisGimbalMsgPayload.h"
#include "architecture/utilities/bskLogging.h"
#include "twoAxisGimbalAxisToMotorAngles/twoAxisGimbalLookupTables.h"

/*! @brief Two-Axis Gimbal Axis-To-Motor Angles Class. */
class TwoAxisGimbalAxisToMotorAngles : public SysModel {
   public:
    TwoAxisGimbalAxisToMotorAngles(const TwoAxisGimbalLookupTables& gimbalLookupTables);  //!< Constructor
    ~TwoAxisGimbalAxisToMotorAngles() = default;                                          //!< Destructor
    void reset(uint64_t currentSimNanos) override;                                        //!< Reset member function
    void updateState(uint64_t currentSimNanos) override;                                  //!< Update member function
    void setDcmMB(
        const Eigen::Matrix3d dcm_MB);        //!< Setter method for dcm_MB (DCM from body frame to gimbal mount frame)
    const Eigen::Matrix3d& getDcmMB() const;  //!< Getter method for dcm_MB (DCM from body frame to gimbal mount frame)

    ReadFunctor<BodyHeadingMsgPayload>
        thrustDirectionInMsg;  //!< Input msg for the requested gimbal body-frame thrust direction vector
    Message<TwoAxisGimbalMsgPayload>
        twoAxisGimbalOutMsg;  //!< Output msg for the corresponding gimbal tip and tilt angles
    Message<HingedRigidBodyMsgPayload> motor1AngleOutMsg;  //!< Output message for the motor 1 angle
    Message<HingedRigidBodyMsgPayload> motor2AngleOutMsg;  //!< Output message for the motor 1 angle

    BSKLogger* bskLogger;  //!< BSK Logging

   private:
    Eigen::Matrix3d dcm_MB;    //!< Attitude DCM for the gimbal mount frame (hub-fixed) relative to the hub body B frame
    double gimbalTipAngle{};   //!< [rad] Gimbal tip angle (sequential angle 1)
    double gimbalTiltAngle{};  //!< [rad] Gimbal tilt angle (sequential angle 2)
    double previousWrittenTime{-1.0};  //!< [s] Time the previous input message was written

    TwoAxisGimbalLookupTables gimbalLookupTables;
};

#endif /* TWOAXISGIMBALAXISTOMOTORANGLES */
