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

/*! Set the solar array drive axis in body frame coordinates.
 *  @param axis [-] solar array drive axis in body frame coordinates
 */
void SolarArrayReference::setA1Hat_B(const Eigen::Vector3f& axis) { this->algorithm.setA1Hat_B(axis); }

/*! Get the solar array drive axis in body frame coordinates.
 *  @return Eigen::Vector3f [-] solar array drive axis in body frame coordinates
 */
Eigen::Vector3f SolarArrayReference::getA1Hat_B() const { return this->algorithm.getA1Hat_B(); }

/*! Set the solar array surface normal at zero rotation.
 *  @param normal [-] solar array surface normal at zero rotation
 */
void SolarArrayReference::setA2Hat_B(const Eigen::Vector3f& normal) { this->algorithm.setA2Hat_B(normal); }

/*! Get the solar array surface normal at zero rotation.
 *  @return Eigen::Vector3f [-] solar array surface normal at zero rotation
 */
Eigen::Vector3f SolarArrayReference::getA2Hat_B() const { return this->algorithm.getA2Hat_B(); }
