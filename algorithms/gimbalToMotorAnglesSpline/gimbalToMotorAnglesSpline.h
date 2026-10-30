#ifndef _GIMBALTOMOTORANGLESSPLINE_
#define _GIMBALTOMOTORANGLESSPLINE_

#include <memory>

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "gimbalToMotorAnglesSplineAlgorithm.h"
#include "msgPayloadDef/MotorAngleRefMsgF32Payload.h"
#include "msgPayloadDef/TwoAxisGimbalMsgF32Payload.h"

/*! @brief Gimbal-to-motor angles spline adapter. Reads the requested gimbal tip and tilt angle message, delegates
the angle computation to GimbalToMotorAnglesSplineAlgorithm, and writes the corresponding stepper motor angles to the
output messages. */
class GimbalToMotorAnglesSpline final : public SysModel {
   public:
    GimbalToMotorAnglesSpline() = default;                //!< Constructor
    ~GimbalToMotorAnglesSpline() override = default;      //!< Destructor
    void reset(uint64_t currentSimNanos) override;        //!< Reset member function
    void updateState(uint64_t currentSimNanos) override;  //!< Update member function
    void reconfigure() const;
    void reInitialize();

    // Phase 1: public configuration properties -- set before reset().
    float stepAngle{};                              //!< [rad/step] angle per motor step (must be set before reset())
    float defaultMotor1Angle{};                     //!< [rad] Motor 1 angle output before the first valid request
    float defaultMotor2Angle{};                     //!< [rad] Motor 2 angle output before the first valid request
    float centerTipAngle{};                         //!< [rad] Gimbal tip angle at the addressable rhombus center
    float centerTiltAngle{};                        //!< [rad] Gimbal tilt angle at the addressable rhombus center
    float halfDiagonalTip{};                        //!< [rad] Half of the rhombus diagonal along the gimbal tip axis
    float halfDiagonalTilt{};                       //!< [rad] Half of the rhombus diagonal along the gimbal tilt axis
    SplineCoefficients motor1SplineCoefficients{};  //!< [steps] Motor 1 step surface control coefficients
    SplineCoefficients motor2SplineCoefficients{};  //!< [steps] Motor 2 step surface control coefficients

    ReadFunctor<TwoAxisGimbalMsgF32Payload>
        twoAxisGimbalInMsg;  //!< Input msg for the corresponding gimbal tip and tilt angles
    Message<MotorAngleRefMsgF32Payload> motor1AngleOutMsg;  //!< Output message for the motor 1 angle
    Message<MotorAngleRefMsgF32Payload> motor2AngleOutMsg;  //!< Output message for the motor 2 angle

   private:
    uint64_t prevInMsgTime = 0; /*!< [ns] Write time of the previous input message*/
    bool hasPrevInMsg = false;  /*!< True after the first input message is read*/
    std::unique_ptr<GimbalToMotorAnglesSplineAlgorithm> algorithm = nullptr;
    GimbalToMotorAnglesSplineConfig toConfig() const;
};

#endif /* _GIMBALTOMOTORANGLESSPLINE_ */
