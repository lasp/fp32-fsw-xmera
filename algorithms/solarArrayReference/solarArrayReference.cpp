#include "solarArrayReference.h"
#include "utilities/xmera/xmeraLifecycleException.h"
#include <stdexcept>

#include <utilities/fsw/eigenSupport.h>

/*! Validate that the required input messages are linked, build the algorithm's configuration from the adapter's
 stored properties, and (re)construct the embedded algorithm.
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void SolarArrayReference::reset(uint64_t callTime) {
    if (!this->sunHeadingInMsg.isLinked()) {
        throw std::invalid_argument("solarArrayReference.sunHeadingInMsg wasn't connected.");
    }
    if (!this->attRefInMsg.isLinked()) {
        throw std::invalid_argument("solarArrayReference.attRefInMsg wasn't connected.");
    }

    const SolarArrayReferenceConfig config =
        SolarArrayReferenceConfig::create(SolarArrayAxes{this->driveAxis, this->surfaceNormal},
                                          this->alignmentThreshold,
                                          this->trackingMode,
                                          this->specifiedArrayAngle,
                                          this->offsetAngle);
    this->algorithm = std::make_unique<SolarArrayReferenceAlgorithm>(config);
}

SolarArrayReferenceConfig SolarArrayReference::toConfig() const {
    return SolarArrayReferenceConfig::create(SolarArrayAxes{this->driveAxis, this->surfaceNormal},
                                             this->alignmentThreshold,
                                             this->trackingMode,
                                             this->specifiedArrayAngle,
                                             this->offsetAngle);
}

void SolarArrayReference::reconfigure() const {
    if (!this->algorithm) {
        throw XmeraLifecycleException("SolarArrayReference reset() has not been called.");
    }
    this->algorithm->setConfig(this->toConfig());
}

/*! Reset the algorithm's retained runtime state without rebuilding its configuration. */
void SolarArrayReference::reInitialize() {
    if (!this->algorithm) {
        throw XmeraLifecycleException("SolarArrayReference reset() has not been called.");
    }
    this->algorithm->reInitialize();
}

/*! Read the Sun heading and reference messages, run the solar array reference law, and write the commanded array
 reference angle output message.
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void SolarArrayReference::updateState(uint64_t callTime) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("SolarArrayReference reset() has not been called.");
    }

    const InertialHeadingMsgF32Payload sunHeadingIn = this->sunHeadingInMsg();
    const AttRefMsgF32Payload attRefIn = this->attRefInMsg();

    const Eigen::Vector3f sigma_RN = cArrayToEigenVector(attRefIn.sigma_RN);
    const Eigen::Vector3f rHat_SB_N = cArrayToEigenVector(sunHeadingIn.rHat_XN_N);

    const float thetaRef = this->algorithm->update(sigma_RN, rHat_SB_N);

    MotorAngleRefMsgF32Payload solarArrayRefOut = {};
    solarArrayRefOut.theta = thetaRef;
    this->solarArrayRefOutMsg.write(solarArrayRefOut, this->moduleID, callTime);
}
