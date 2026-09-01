#ifndef _GIMBALANGLESTOMOTORANGLES_
#define _GIMBALANGLESTOMOTORANGLES_

#include <array>
#include <memory>
#include <numbers>

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "gimbalAnglesToMotorAnglesAlgorithm.h"
#include "msgPayloadDef/HingedRigidBodyMsgF32Payload.h"
#include "msgPayloadDef/TwoAxisGimbalMsgF32Payload.h"

/*! @brief Gimbal Angles-To-Motor Angles adapter. Reads the requested gimbal tip and tilt angle
message, delegates the angle computation to GimbalAnglesToMotorAnglesAlgorithm, and writes the
corresponding stepper motor angles to the output messages. */
class GimbalAnglesToMotorAngles final : public SysModel {
   public:
    GimbalAnglesToMotorAngles() = default;                //!< Constructor
    ~GimbalAnglesToMotorAngles() override = default;      //!< Destructor
    void reset(uint64_t currentSimNanos) override;        //!< Reset member function
    void updateState(uint64_t currentSimNanos) override;  //!< Update member function
    void reconfigure() const;
    void reInitialize();

    // Phase 1: public configuration properties -- set before reset().
    float minAngle{0.0F};                              //!< [rad] lower bound of the motor travel range
    float maxAngle{2.0F * std::numbers::pi_v<float>};  //!< [rad] upper bound of the motor travel range
    std::array<float, NUM_GIMBAL_TO_MOTOR_TABLE_ELEMENTS>
        gimbalToMotor1AngleData;  //!< [rad] Gimbal-to-motor 1 angle interpolation table
    std::array<float, NUM_GIMBAL_TO_MOTOR_TABLE_ELEMENTS>
        gimbalToMotor2AngleData;  //!< [rad] Gimbal-to-motor 2 angle interpolation table
    std::array<int, NUM_GIMBAL_TO_MOTOR_TABLE_ROWS>
        rowStartStrideIndices;  //!< [-] Stride indices for the starting location of the table rows
    std::array<int, NUM_GIMBAL_TO_MOTOR_TABLE_ROWS>
        rowStartColIndices;  //!< [-] Column indices for the starting location of the table rows
    int tipColIdxOffset;     //!< [-] Table column index corresponding to zero tip angle
    int tiltRowIdxOffset;    //!< [-] Table row index corresponding to zero tilt angle
    float tableStepAngle;    //!< [rad] Interpolation table motor discretization step

    ReadFunctor<TwoAxisGimbalMsgF32Payload>
        twoAxisGimbalInMsg;  //!< Input msg for the corresponding gimbal tip and tilt angles
    Message<HingedRigidBodyMsgF32Payload> motor1AngleOutMsg;  //!< Output message for the motor 1 angle
    Message<HingedRigidBodyMsgF32Payload> motor2AngleOutMsg;  //!< Output message for the motor 2 angle

   private:
    double previousWrittenTime{-1.0};  //!< [s] Time the previous input message was written
    std::unique_ptr<GimbalAnglesToMotorAnglesAlgorithm> algorithm = nullptr;
    GimbalAnglesToMotorAnglesConfig toConfig() const;
};

#endif /* GIMBALANGLESTOMOTORANGLES */
