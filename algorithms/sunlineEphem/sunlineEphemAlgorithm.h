#ifndef F32XMERA_SUNLINE_EPHEM_ALGORITHM_H
#define F32XMERA_SUNLINE_EPHEM_ALGORITHM_H

#include <Eigen/Core>

class SunlineEphemAlgorithm {
   public:
    Eigen::Vector3f updateState(const Eigen::Vector3d& rSun,
                                const Eigen::Vector3d& rSc,
                                const Eigen::Vector3f& sigma_BN) const;
};

#endif
