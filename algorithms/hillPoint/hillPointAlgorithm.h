// SPDX-FileCopyrightText: 2025 Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
// SPDX-License-Identifier: MIT

#ifndef F32XMERA_HILL_POINT_ALGORITHM_H
#define F32XMERA_HILL_POINT_ALGORITHM_H

#include "hillPointTypes.h"
#include <Eigen/Core>

class HillPointAlgorithm final {
   public:
    HillPointAlgorithm() = default;

    HillPointOutput update(const Eigen::Vector3d& r_BN_N,
                           const Eigen::Vector3d& v_BN_N,
                           const Eigen::Vector3d& r_planet_N,
                           const Eigen::Vector3d& v_planet_N) const;
};

#endif
