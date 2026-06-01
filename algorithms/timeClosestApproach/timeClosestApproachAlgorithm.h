// SPDX-License-Identifier: ISC
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef F32XMERA_TIME_CA_ALGORITHM_H
#define F32XMERA_TIME_CA_ALGORITHM_H

#include <Eigen/Core>

struct TimeClosestApproachOutput {
    float tCA{};
    float sigmaTca{};
};

/*! @brief Pure computation for time of closest approach estimation during a rectilinear flyby. */
class TimeClosestApproachAlgorithm {
   public:
    static TimeClosestApproachOutput update(int numberOfStates,
                                            const Eigen::Vector3f& r_BN_N,
                                            const Eigen::Vector3f& v_BN_N,
                                            const Eigen::MatrixXf& filterCovariance);
};

#endif
