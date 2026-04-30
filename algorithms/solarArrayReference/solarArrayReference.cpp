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

    this->algorithm.reset();
}

void SolarArrayReference::updateState(uint64_t callTime) {
    NavAttMsgPayload attNavIn = this->attNavInMsg();
    AttRefMsgPayload attRefIn = this->attRefInMsg();
    HingedRigidBodyMsgPayload hingedRigidBodyIn = this->hingedRigidBodyInMsg();

    const Eigen::Vector3d sigma_BN = cArrayToEigenVector(attNavIn.sigma_BN);
    const Eigen::Vector3d sigma_RN = cArrayToEigenVector(attRefIn.sigma_RN);
    const Eigen::Vector3d vehSunPntBdy = cArrayToEigenVector(attNavIn.vehSunPntBdy);

    const SolarArrayReferenceOutput output =
        this->algorithm.update(sigma_BN, sigma_RN, vehSunPntBdy, hingedRigidBodyIn.theta, callTime);

    HingedRigidBodyMsgPayload hingedRigidBodyRefOut = {};
    hingedRigidBodyRefOut.theta = output.theta;
    hingedRigidBodyRefOut.thetaDot = output.thetaDot;
    this->hingedRigidBodyRefOutMsg.write(&hingedRigidBodyRefOut, this->moduleID, callTime);
}

/*! Set the solar array drive axis in body frame coordinates.
 *  @param axis [-] solar array drive axis in body frame coordinates
 */
void SolarArrayReference::setA1Hat_B(const Eigen::Vector3d& axis) { this->algorithm.setA1Hat_B(axis); }

/*! Get the solar array drive axis in body frame coordinates.
 *  @return Eigen::Vector3d [-] solar array drive axis in body frame coordinates
 */
Eigen::Vector3d SolarArrayReference::getA1Hat_B() const { return this->algorithm.getA1Hat_B(); }

/*! Set the solar array surface normal at zero rotation.
 *  @param normal [-] solar array surface normal at zero rotation
 */
void SolarArrayReference::setA2Hat_B(const Eigen::Vector3d& normal) { this->algorithm.setA2Hat_B(normal); }

/*! Get the solar array surface normal at zero rotation.
 *  @return Eigen::Vector3d [-] solar array surface normal at zero rotation
 */
Eigen::Vector3d SolarArrayReference::getA2Hat_B() const { return this->algorithm.getA2Hat_B(); }

/*! Set the attitude frame flag.
 *  @param frame attitude frame flag (0 = referenceFrame, 1 = bodyFrame)
 */
void SolarArrayReference::setAttitudeFrame(const int frame) { this->algorithm.setAttitudeFrame(frame); }

/*! Get the attitude frame flag.
 *  @return int attitude frame flag (0 = referenceFrame, 1 = bodyFrame)
 */
int SolarArrayReference::getAttitudeFrame() const { return this->algorithm.getAttitudeFrame(); }
