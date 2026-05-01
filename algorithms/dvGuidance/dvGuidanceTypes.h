#ifndef F32XMERA_DV_GUIDANCE_TYPES_H
#define F32XMERA_DV_GUIDANCE_TYPES_H

#include <Eigen/Core>

struct DvGuidanceOutput {
    Eigen::Vector3d sigma_RN = Eigen::Vector3d::Zero();
    Eigen::Vector3d omega_RN_N = Eigen::Vector3d::Zero();
    Eigen::Vector3d domega_RN_N = Eigen::Vector3d::Zero();
};

#endif
