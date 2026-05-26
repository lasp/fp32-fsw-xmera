#ifndef F32XMERA_HILL_POINT_ALGORITHM_H
#define F32XMERA_HILL_POINT_ALGORITHM_H

#include "hillPointTypes.h"
#include <Eigen/Core>

class HillPointAlgorithm final {
   public:
    explicit HillPointAlgorithm(const HillPointConfig& config);

    void setConfig(const HillPointConfig& config);

    HillPointOutput update(const Eigen::Vector3d& r_BN_N,
                           const Eigen::Vector3d& v_BN_N,
                           const Eigen::Vector3d& r_planet_N,
                           const Eigen::Vector3d& v_planet_N) const;

   private:
    HillPointConfig cfg;
};

#endif
