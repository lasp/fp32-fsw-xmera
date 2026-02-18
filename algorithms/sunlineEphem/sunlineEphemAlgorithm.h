#ifndef F32XMERA_SUNLINE_EPHEM_ALGORITHM_H
#define F32XMERA_SUNLINE_EPHEM_ALGORITHM_H

#include <Eigen/Core>

class SunlineEphemAlgorithm {
   public:
    Eigen::Vector3f updateState(const Eigen::Vector3d& r_SN_N,
                                const Eigen::Vector3d& r_BN_N,
                                const Eigen::Vector3f& sigma_BN) const;
};

#endif
