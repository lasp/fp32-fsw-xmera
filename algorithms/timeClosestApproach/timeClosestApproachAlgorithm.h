// SPDX-License-Identifier: ISC
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef F32XMERA_TIME_CA_ALGORITHM_H
#define F32XMERA_TIME_CA_ALGORITHM_H

#include <Eigen/Core>

struct TimeClosestApproachOutput {
    float tCA;       //!< the predicted time of closest approach [s]
    float sigmaTca;  //!< the predicted time of closest approach standard deviation [s]
};

// timeClosestApproach has no tunable parameters; the Config class is intentionally empty so the
// algorithm can still follow the standard two-phase init pattern.
class TimeClosestApproachConfig final {
   public:
    static TimeClosestApproachConfig create() { return {}; }

   private:
    TimeClosestApproachConfig() = default;
};

class TimeClosestApproachAlgorithm final {
   public:
    explicit TimeClosestApproachAlgorithm(const TimeClosestApproachConfig& config);

    void setConfig(const TimeClosestApproachConfig& config);

    // NOLINTNEXTLINE(readability-convert-member-functions-to-static)
    TimeClosestApproachOutput update(const Eigen::Vector3d& r_BN_N,
                                     const Eigen::Vector3d& v_BN_N,
                                     const Eigen::Matrix<float, 6, 6>& filterCovariance) const;

   private:
    TimeClosestApproachConfig cfg;
};

#endif
