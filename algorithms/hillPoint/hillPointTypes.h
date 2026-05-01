// SPDX-FileCopyrightText: 2025 Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
// SPDX-License-Identifier: MIT

#ifndef F32XMERA_HILL_POINT_TYPES_H
#define F32XMERA_HILL_POINT_TYPES_H

#include <Eigen/Core>

struct HillPointOutput {
    Eigen::Vector3d sigma_RN = Eigen::Vector3d::Zero();
    Eigen::Vector3d omega_RN_N = Eigen::Vector3d::Zero();
    Eigen::Vector3d domega_RN_N = Eigen::Vector3d::Zero();
};

#endif
