// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "mrpPD.h"
#include "utilities/freestandingInvalidArgument.h"
#include <architecture/utilities/eigenSupport.h>

void MrpPD::reset(uint64_t callTime) {
    if (!this->guidInMsg.isLinked()) {
        throw std::invalid_argument("mrpPD.guidInMsg wasn't connected.");
    }
    if (!this->vehConfigInMsg.isLinked()) {
        throw std::invalid_argument("mrpPD.vehConfigInMsg wasn't connected.");
    }

    if (this->vehConfigInMsg.isWritten()) {
        const auto vehicleConfigInMsg = this->vehConfigInMsg();
        this->spacecraftInertia = cArrayToEigenMatrix3(vehicleConfigInMsg.ISCPntB_B);
    }
    this->rebuildAlgorithmConfig();
}

void MrpPD::updateState(uint64_t callTime) {
    auto torqueCmdMsgF32Payload = CmdTorqueBodyMsgF32Payload();
    if (this->guidInMsg.isWritten()) {
        const auto localGuidInMsg = this->guidInMsg();
        const Eigen::Vector3f sigma_BR = cArrayToEigenVector(localGuidInMsg.sigma_BR);
        const Eigen::Vector3f omega_BR_B = cArrayToEigenVector(localGuidInMsg.omega_BR_B);
        const Eigen::Vector3f domega_RN_B = cArrayToEigenVector(localGuidInMsg.domega_RN_B);

        const auto torque = this->algorithm.update(sigma_BR, omega_BR_B, domega_RN_B);
        eigenVectorToCArray(torque, torqueCmdMsgF32Payload.torqueRequestBody);
    }

    this->cmdTorqueOutMsg.write(&torqueCmdMsgF32Payload, moduleID, callTime);
}

void MrpPD::setK(float K) {
    if (!MrpPDConfig::isValidProportionalGainK(K)) {
        FSW_THROW_INVALID_ARGUMENT("mrpPD: K must be non-negative");
    }
    this->K = K;
    this->rebuildAlgorithmConfig();
}

float MrpPD::getK() const { return this->K; }

void MrpPD::setP(float P) {
    if (!MrpPDConfig::isValidDerivativeGainP(P)) {
        FSW_THROW_INVALID_ARGUMENT("mrpPD: P must be non-negative");
    }
    this->P = P;
    this->rebuildAlgorithmConfig();
}

float MrpPD::getP() const { return this->P; }

void MrpPD::setKnownTorquePntB_B(const Eigen::Vector3f& knownTorquePntB_B) {
    if (!MrpPDConfig::isValidKnownTorquePntB_B(knownTorquePntB_B)) {
        FSW_THROW_INVALID_ARGUMENT("mrpPD: knownTorquePntB_B must be finite");
    }
    this->knownTorquePntB_B = knownTorquePntB_B;
    this->rebuildAlgorithmConfig();
}

const Eigen::Vector3f& MrpPD::getKnownTorquePntB_B() const { return this->knownTorquePntB_B; }

void MrpPD::rebuildAlgorithmConfig() {
    const MrpPDConfig cfg = MrpPDConfig::create(this->K, this->P, this->knownTorquePntB_B, this->spacecraftInertia);
    this->algorithm.setConfig(cfg);
}
