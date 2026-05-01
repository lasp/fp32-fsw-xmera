// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef XMERAF32_MRP_PD_ALGORITHM_H
#define XMERAF32_MRP_PD_ALGORITHM_H

#include "mrpPDTypes.h"
#include <Eigen/Core>

class MrpPDAlgorithm final {
   public:
    explicit MrpPDAlgorithm(const MrpPDConfig& config) : cfg(config) {}

    void setConfig(const MrpPDConfig& config) { this->cfg = config; }

    Eigen::Vector3f update(const Eigen::Vector3f& sigma_BR,
                           const Eigen::Vector3f& omega_BR_B,
                           const Eigen::Vector3f& domega_RN_B) const;

   private:
    MrpPDConfig cfg;
};

#endif
