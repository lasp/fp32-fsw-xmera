#include "solarArrayReference.h"
#include <stdexcept>

#include <architecture/utilities/eigenSupport.h>

void SolarArrayReference::reset(uint64_t callTime) {
    if (!this->attNavInMsg.isLinked()) {
        throw std::invalid_argument("solarArrayReference.attNavInMsg wasn't connected.");
    }
    if (!this->attRefInMsg.isLinked()) {
        throw std::invalid_argument("solarArrayReference.attRefInMsg wasn't connected.");
    }
    if (!this->hingedRigidBodyInMsg.isLinked()) {
        throw std::invalid_argument("solarArrayReference.hingedRigidBodyInMsg wasn't connected.");
    }
}

void SolarArrayReference::updateState(uint64_t callTime) {
    NavAttMsgF32Payload attNavIn = this->attNavInMsg();
    AttRefMsgF32Payload attRefIn = this->attRefInMsg();
    HingedRigidBodyMsgF32Payload hingedRigidBodyIn = this->hingedRigidBodyInMsg();

    const Eigen::Vector3f sigma_BN = cArrayToEigenVector(attNavIn.sigma_BN);
    const Eigen::Vector3f sigma_RN = cArrayToEigenVector(attRefIn.sigma_RN);
    const Eigen::Vector3f vehSunPntBdy = cArrayToEigenVector(attNavIn.vehSunPntBdy);

    const float thetaRef = this->algorithm.update(sigma_BN, sigma_RN, vehSunPntBdy, hingedRigidBodyIn.theta);

    HingedRigidBodyMsgF32Payload hingedRigidBodyRefOut = {};
    hingedRigidBodyRefOut.theta = thetaRef;
    this->hingedRigidBodyRefOutMsg.write(&hingedRigidBodyRefOut, this->moduleID, callTime);
}

/*! Set the solar array drive axis and surface normal in body frame coordinates.
 *  @param driveAxis [-] solar array drive axis in body frame coordinates
 *  @param surfaceNormal [-] solar array surface normal at zero rotation
 */
void SolarArrayReference::setSolarArrayAxes_B(const Eigen::Vector3f& driveAxis,
                                              const Eigen::Vector3f& surfaceNormal) {
    this->algorithm.setSolarArrayAxes_B(driveAxis, surfaceNormal);
}

/*! Get the solar array drive axis and surface normal in body frame coordinates.
 *  @return std::array<Eigen::Vector3f, 2> [driveAxis, surfaceNormal]
 */
std::array<Eigen::Vector3f, 2> SolarArrayReference::getSolarArrayAxes_B() const {
    return this->algorithm.getSolarArrayAxes_B();
}

/*! Set the alignment threshold angle between sun direction and drive axis.
 *  @param threshold [rad] angle threshold in [0, pi/2]
 */
void SolarArrayReference::setAlignmentThreshold(const float threshold) {
    this->algorithm.setAlignmentThreshold(threshold);
}

/*! Get the alignment threshold angle between sun direction and drive axis.
 *  @return float [rad] alignment threshold
 */
float SolarArrayReference::getAlignmentThreshold() const { return this->algorithm.getAlignmentThreshold(); }

/*! Set the tracking mode of the solar array.
 *  @param mode tracking mode (AUTO_TRACK or SPECIFIED_ANGLE)
 */
void SolarArrayReference::setTrackingMode(const TrackingMode mode) { this->algorithm.setTrackingMode(mode); }

/*! Get the tracking mode of the solar array.
 *  @return TrackingMode current tracking mode
 */
TrackingMode SolarArrayReference::getTrackingMode() const { return this->algorithm.getTrackingMode(); }

/*! Set the specified reference array angle (used when trackingMode is SPECIFIED_ANGLE).
 *  @param angle [rad] specified reference array angle
 */
void SolarArrayReference::setSpecifiedArrayAngle(const float angle) {
    this->algorithm.setSpecifiedArrayAngle(angle);
}

/*! Get the specified reference array angle.
 *  @return float [rad] specified reference array angle
 */
float SolarArrayReference::getSpecifiedArrayAngle() const { return this->algorithm.getSpecifiedArrayAngle(); }
