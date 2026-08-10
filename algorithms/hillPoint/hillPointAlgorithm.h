#ifndef F32XMERA_HILL_POINT_ALGORITHM_H
#define F32XMERA_HILL_POINT_ALGORITHM_H

#include <Eigen/Core>

struct HillPointOutput {
    Eigen::Vector3f sigma_RN = Eigen::Vector3f::Zero();
    Eigen::Vector3f omega_RN_N = Eigen::Vector3f::Zero();
    Eigen::Vector3f domega_RN_N = Eigen::Vector3f::Zero();
};

class HillPointAlgorithm final {
   public:
    static HillPointOutput update(const Eigen::Vector3d& r_BN_N,
                                  const Eigen::Vector3d& v_BN_N,
                                  const Eigen::Vector3d& r_PN_N,
                                  const Eigen::Vector3d& v_PN_N);
};

#endif
