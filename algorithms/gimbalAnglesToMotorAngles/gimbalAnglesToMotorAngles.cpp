#include "gimbalAnglesToMotorAngles.h"

#include <memory>
#include <stdexcept>

#include "utilities/xmera/xmeraLifecycleException.h"

/*! This method checks the input message to ensure it is linked and builds the algorithm from the
configured parameters.
 @return void
 @param currentSimNanos [ns] Time the method is called
*/
void GimbalAnglesToMotorAngles::reset(uint64_t currentSimNanos) {
    if (!this->twoAxisGimbalInMsg.isLinked()) {
        throw std::invalid_argument("gimbalAnglesToMotorAngles.twoAxisGimbalInMsg wasn't connected.");
    }

    this->algorithm = std::make_unique<GimbalAnglesToMotorAnglesAlgorithm>(this->toConfig());
    this->prevInMsgTime = 0U;
    this->hasPrevInMsg = false;
}

/*! This method builds a validated configuration from the public configuration properties.
 @return GimbalAnglesToMotorAnglesConfig
*/
GimbalAnglesToMotorAnglesConfig GimbalAnglesToMotorAngles::toConfig() const {
    return GimbalAnglesToMotorAnglesConfig::create(this->stepsPerRevolution,
                                                   GimbalRhombusDomain{.halfDiagonalTip = this->halfDiagonalTip,
                                                                       .halfDiagonalTilt = this->halfDiagonalTilt,
                                                                       .thetaMax = this->thetaMax},
                                                   this->motor1SplineCoefficients,
                                                   this->motor2SplineCoefficients);
}

/*! This method replaces the algorithm configuration with the current public configuration properties.
 The previous valid motor angles are kept. If there are none, the motor angles of the new configuration at zero
 gimbal angles are used at once.
 @return void
*/
void GimbalAnglesToMotorAngles::reconfigure() const {
    if (!this->algorithm) {
        throw XmeraLifecycleException("GimbalAnglesToMotorAngles reset() has not been called.");
    }

    this->algorithm->setConfig(this->toConfig());
}

/*! This method clears the stored motor angles, so the motor angles of the current configuration at zero gimbal
 angles are used until the next valid request.
 @return void
*/
void GimbalAnglesToMotorAngles::reInitialize() {
    if (!this->algorithm) {
        throw XmeraLifecycleException("GimbalAnglesToMotorAngles reset() has not been called.");
    }
    this->algorithm->reInitialize();
}

/*! This method reads the incoming requested gimbal angles, delegates the motor angle computation to the
algorithm, and writes the resulting motor angles to the output messages.
 @return void
 @param currentSimNanos [ns] The current time of simulation
*/
void GimbalAnglesToMotorAngles::updateState(uint64_t currentSimNanos) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("GimbalAnglesToMotorAngles reset() has not been called.");
    }

    // Analyze the input message time. A message written at t = 0 is new if no message has been read yet.
    if (!this->twoAxisGimbalInMsg.isWritten()) {
        return;
    }
    const uint64_t writeTime = this->twoAxisGimbalInMsg.timeWritten();
    if (this->hasPrevInMsg && writeTime == this->prevInMsgTime) {
        return;
    }
    this->hasPrevInMsg = true;
    this->prevInMsgTime = writeTime;

    // Store the commanded gimbal tip and tilt angles
    const auto twoAxisGimbalIn = this->twoAxisGimbalInMsg();
    const float gimbalAngle1 = twoAxisGimbalIn.theta1;  // tip, alpha
    const float gimbalAngle2 = twoAxisGimbalIn.theta2;  // tilt, beta

    // Determine motor angles corresponding to the gimbal angles
    const GimbalAnglesToMotorAnglesOutput motorAngles = this->algorithm->update(gimbalAngle1, gimbalAngle2);

    // Write the module output messages
    auto motor1AngleOut = MotorAngleRefMsgF32Payload();
    motor1AngleOut.theta = motorAngles.motorAngle1;
    this->motor1AngleOutMsg.write(motor1AngleOut, moduleID, currentSimNanos);

    auto motor2AngleOut = MotorAngleRefMsgF32Payload();
    motor2AngleOut.theta = motorAngles.motorAngle2;
    this->motor2AngleOutMsg.write(motor2AngleOut, moduleID, currentSimNanos);
}
