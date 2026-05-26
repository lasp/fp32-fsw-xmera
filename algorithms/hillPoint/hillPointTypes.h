#ifndef F32XMERA_HILL_POINT_TYPES_H
#define F32XMERA_HILL_POINT_TYPES_H

#include <Eigen/Core>

struct HillPointOutput {
    Eigen::Vector3f sigma_RN = Eigen::Vector3f::Zero();
    Eigen::Vector3f omega_RN_N = Eigen::Vector3f::Zero();
    Eigen::Vector3f domega_RN_N = Eigen::Vector3f::Zero();
};

#endif
