// SPDX-License-Identifier: ISC
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef F32XMERA_TIME_CA_ALGORITHM_H
#define F32XMERA_TIME_CA_ALGORITHM_H

#include <Eigen/Core>

struct TimeClosestApproachOutput {
    float tCA;       //!< the predicted time of closest approach [s]
    float sigmaTca;  //!< the predicted time of closest approach standard deviation [s]
};

class TimeClosestApproachAlgorithm {
   public:
    static TimeClosestApproachOutput update(const Eigen::Vector3d& r_BN_N,
                                            const Eigen::Vector3d& v_BN_N,
                                            const Eigen::MatrixXf& filterCovariance);
};

#endif
