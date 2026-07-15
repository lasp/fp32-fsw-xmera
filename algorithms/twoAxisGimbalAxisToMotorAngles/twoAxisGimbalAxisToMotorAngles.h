// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef _TWOAXISGIMBALAXISTOMOTORANGLES_
#define _TWOAXISGIMBALAXISTOMOTORANGLES_

#include <Eigen/Core>
#include <array>
#include <numbers>

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDef/BodyHeadingMsgPayload.h"
#include "architecture/msgPayloadDef/HingedRigidBodyMsgPayload.h"
#include "architecture/msgPayloadDef/TwoAxisGimbalMsgPayload.h"
#include "architecture/utilities/bskLogging.h"

#define NUM_GIMBAL_TO_MOTOR_TABLE_ROWS 111
#define NUM_GIMBAL_TO_MOTOR_TABLE_COLS 76

const double DEG2RAD = M_PI / 180.0;

enum class FixedAngle { ANGLE_1_FIXED, ANGLE_2_FIXED };

struct MotorAngles {
    double angle1;
    double angle2;
    bool isValidInterpolation;
};

/*! @brief Two-Axis Gimbal Axis-To-Motor Angles Class. */
class TwoAxisGimbalAxisToMotorAngles : public SysModel {
   public:
    TwoAxisGimbalAxisToMotorAngles(
        const std::array<std::array<double, NUM_GIMBAL_TO_MOTOR_TABLE_COLS>, NUM_GIMBAL_TO_MOTOR_TABLE_ROWS>&
            gimbalToMotor1Data,
        const std::array<std::array<double, NUM_GIMBAL_TO_MOTOR_TABLE_COLS>,
                         NUM_GIMBAL_TO_MOTOR_TABLE_ROWS>& gimbalToMotor2Data);  //!< Constructor
    ~TwoAxisGimbalAxisToMotorAngles() = default;                                //!< Destructor
    void reset(uint64_t currentSimNanos) override;                              //!< Reset member function
    void updateState(uint64_t currentSimNanos) override;                        //!< Update member function
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

    MotorAngles gimbalAnglesToMotorAngles(double gimbalTipAngle,
                                          double gimbalTiltAngle);  //!< Method to determine the stepper motor angles
    //!< given the gimbal sequential tip and tilt angles
    MotorAngles pullAngles(double gimbalAngle1, double gimbalAngle2) const;
    bool bilinearInterpolationRequired(
        double gimbalAngle1,
        double gimbalAngle2);  //!< Method to determine if bilinear interpolation is required
    bool noInterpolationRequired(double gimbalAngle1,
                                 double gimbalAngle2);  //!< Method to determine if no interpolation is required
    bool linearInterpolationRequired(double angle);     //!< Method to determine if linear interpolation is required
    MotorAngles bilinearlyInterpolateAngles(double gimbalAngle1, double gimbalAngle2);
    MotorAngles linearlyInterpolateAngles(double gimbalAngle1, double gimbalAngle2, FixedAngle fixedAngle);

    double tableStepAngle{0.5 * DEG2RAD};  //!< [rad] Interpolation table motor discretization angle
    std::array<std::array<double, NUM_GIMBAL_TO_MOTOR_TABLE_COLS>, NUM_GIMBAL_TO_MOTOR_TABLE_ROWS>
        gimbalAnglesToMotor1AngleData;  //!< [rad] Gimbal-to-motor 1 angle interpolation table storage array
    std::array<std::array<double, NUM_GIMBAL_TO_MOTOR_TABLE_COLS>, NUM_GIMBAL_TO_MOTOR_TABLE_ROWS>
        gimbalAnglesToMotor2AngleData;  //!< [rad] Gimbal-to-motor 2 angle interpolation table storage array
};

#endif /* TWOAXISGIMBALAXISTOMOTORANGLES */
