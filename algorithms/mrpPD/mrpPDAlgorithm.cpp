// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "mrpPDAlgorithm.h"

Eigen::Vector3f MrpPDAlgorithm::update(const Eigen::Vector3f& sigma_BR,
                                       const Eigen::Vector3f& omega_BR_B,
                                       const Eigen::Vector3f& domega_RN_B) const {
    // L_r = -K sigma_BR - P omega_BR_B + [I] domega_RN_B - L_known   [N*m]
    return -this->cfg.getProportionalGainK() * sigma_BR - this->cfg.getDerivativeGainP() * omega_BR_B +
           this->cfg.getSpacecraftInertia() * domega_RN_B - this->cfg.getKnownTorquePntB_B();
}
