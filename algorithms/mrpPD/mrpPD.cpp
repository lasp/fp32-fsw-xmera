// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "mrpPD.h"

#include "utilities/xmeraLifecycleException.h"
#include <architecture/utilities/eigenSupport.h>
#include <stdexcept>

void MrpPD::reset(const uint64_t callTime) {
    if (!this->guidInMsg.isLinked()) {
        throw std::invalid_argument("mrpPD.guidInMsg wasn't connected.");
    }
    if (!this->vehConfigInMsg.isLinked()) {
        throw std::invalid_argument("mrpPD.vehConfigInMsg wasn't connected.");
    }

    const VehicleConfigMsgF32Payload sc = this->vehConfigInMsg();
    const Eigen::Matrix3f spacecraftInertia = cArrayToEigenMatrix3(sc.ISCPntB_B);

    auto config = MrpPDConfig::create(this->K, this->P, this->knownTorquePntB_B, spacecraftInertia);
    this->algorithm = std::make_unique<MrpPDAlgorithm>(config);
}

void MrpPD::updateState(const uint64_t callTime) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("MrpPD reset() has not been called.");
    }

    const auto [sigma_BR_arr, omega_BR_B_arr, omega_RN_B_arr, domega_RN_B_arr] = this->guidInMsg();
    const Eigen::Vector3f sigma_BR = cArrayToEigenVector(sigma_BR_arr);
    const Eigen::Vector3f omega_BR_B = cArrayToEigenVector(omega_BR_B_arr);
    const Eigen::Vector3f domega_RN_B = cArrayToEigenVector(domega_RN_B_arr);

    const Eigen::Vector3f torque = this->algorithm->update(sigma_BR, omega_BR_B, domega_RN_B);

    CmdTorqueBodyMsgF32Payload out{};
    eigenVectorToCArray(torque, out.torqueRequestBody);
    this->cmdTorqueOutMsg.write(&out, this->moduleID, callTime);
}
