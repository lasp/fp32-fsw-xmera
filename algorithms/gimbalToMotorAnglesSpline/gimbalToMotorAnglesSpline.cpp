#include "gimbalToMotorAnglesSpline.h"

#include <memory>
#include <stdexcept>

#include "utilities/xmera/xmeraLifecycleException.h"

/*! This method checks the input message to ensure it is linked and builds the algorithm from the
configured parameters.
 @return void
 @param currentSimNanos [ns] Time the method is called
*/
void GimbalToMotorAnglesSpline::reset(uint64_t currentSimNanos) {
    if (!this->twoAxisGimbalInMsg.isLinked()) {
        throw std::invalid_argument("gimbalToMotorAnglesSpline.twoAxisGimbalInMsg wasn't connected.");
    }

    this->algorithm = std::make_unique<GimbalToMotorAnglesSplineAlgorithm>(this->toConfig());
    this->prevInMsgTime = 0U;
    this->hasPrevInMsg = false;
}

/*! This method builds a validated configuration from the public configuration properties.
 @return GimbalToMotorAnglesSplineConfig
*/
GimbalToMotorAnglesSplineConfig GimbalToMotorAnglesSpline::toConfig() const {
    return GimbalToMotorAnglesSplineConfig::create(this->stepAngle,
                                                   this->defaultMotor1Angle,
                                                   this->defaultMotor2Angle,
                                                   GimbalRhombusDomain{.centerTipAngle = this->centerTipAngle,
                                                                       .centerTiltAngle = this->centerTiltAngle,
                                                                       .halfDiagonalTip = this->halfDiagonalTip,
                                                                       .halfDiagonalTilt = this->halfDiagonalTilt},
                                                   this->motor1SplineCoefficients,
                                                   this->motor2SplineCoefficients);
}

/*! This method replaces the algorithm configuration with the current public configuration properties.
 The previous valid motor angles are kept; new default motor angles are used at the next reInitialize().
 @return void
*/
void GimbalToMotorAnglesSpline::reconfigure() const {
    if (!this->algorithm) {
        throw XmeraLifecycleException("GimbalToMotorAnglesSpline reset() has not been called.");
    }

    this->algorithm->setConfig(this->toConfig());
}

/*! This method resets the stored motor angles to the configured default motor angles.
 @return void
*/
void GimbalToMotorAnglesSpline::reInitialize() {
    if (!this->algorithm) {
        throw XmeraLifecycleException("GimbalToMotorAnglesSpline reset() has not been called.");
    }
    this->algorithm->reInitialize();
}

/*! This method reads the incoming requested gimbal angles, delegates the motor angle computation to the
algorithm, and writes the resulting motor angles to the output messages.
 @return void
 @param currentSimNanos [ns] The current time of simulation
*/
void GimbalToMotorAnglesSpline::updateState(uint64_t currentSimNanos) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("GimbalToMotorAnglesSpline reset() has not been called.");
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
    const GimbalToMotorAnglesSplineOutput motorAngles = this->algorithm->update(gimbalAngle1, gimbalAngle2);

    // Write the module output messages
    auto motor1AngleOut = MotorAngleRefMsgF32Payload();
    motor1AngleOut.theta = motorAngles.motorAngle1;
    this->motor1AngleOutMsg.write(motor1AngleOut, moduleID, currentSimNanos);

    auto motor2AngleOut = MotorAngleRefMsgF32Payload();
    motor2AngleOut.theta = motorAngles.motorAngle2;
    this->motor2AngleOutMsg.write(motor2AngleOut, moduleID, currentSimNanos);
}
