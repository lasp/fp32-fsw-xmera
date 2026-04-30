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

    this->algorithm.a1Hat_B = this->a1Hat_B;
    this->algorithm.a2Hat_B = this->a2Hat_B;
    this->algorithm.attitudeFrame = this->attitudeFrame;
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
