// SPDX-License-Identifier: ISC
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "triad.h"
#include <architecture/utilities/eigenSupport.h>
#include <Eigen/Core>
#include <stdexcept>

class XmeraLifecycleException : public std::runtime_error {
   public:
    using runtime_error::runtime_error;
};

void Triad::reset(const uint64_t callTime) {
    if (!this->attNavInMsg.isLinked()) {
        throw std::invalid_argument("triad.attNavInMsg wasn't connected.");
    }
    if (!this->bodyHeadingInMsg.isLinked()) {
        throw std::invalid_argument("triad.bodyHeadingInMsg wasn't connected.");
    }

    auto config = TriadConfig::create(this->sadaHat_B, this->thrustReqHat_N, this->signOfZHat_N);
    this->algorithm = std::make_unique<TriadAlgorithm>(config);
}

TriadConfig Triad::toConfig() const {
    return TriadConfig::create(this->sadaHat_B, this->thrustReqHat_N, this->signOfZHat_N);
}

void Triad::reconfigure() const {
    if (!this->algorithm) {
        throw XmeraLifecycleException("Triad reset() has not been called.");
    }

    this->algorithm->setConfig(this->toConfig());
}

void Triad::updateState(const uint64_t callTime) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("Triad reset() has not been called.");
    }

    const NavAttMsgF32Payload attNavIn = this->attNavInMsg();
    const Eigen::Vector3f sigma_BN = cArrayToEigenVector(attNavIn.sigma_BN);
    const Eigen::Vector3f rHat_SB_B = cArrayToEigenVector(attNavIn.vehSunPntBdy).normalized();

    const BodyHeadingMsgF32Payload bodyHeadingIn = this->bodyHeadingInMsg();
    const Eigen::Vector3f thrustHat_B = cArrayToEigenVector(bodyHeadingIn.rHat_XB_B).normalized();

    const Eigen::Vector3f sigma_RN = this->algorithm->update(sigma_BN, rHat_SB_B, thrustHat_B);

    AttRefMsgF32Payload attRefOut = {};
    eigenVectorToCArray(sigma_RN, attRefOut.sigma_RN);
    this->attRefOutMsg.write(attRefOut, this->moduleID, callTime);
}
