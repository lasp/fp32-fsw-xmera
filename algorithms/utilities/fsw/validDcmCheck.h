#ifndef FP32_XMERA_FSW_ALGORITHMS_VALID_DCM_H
#define FP32_XMERA_FSW_ALGORITHMS_VALID_DCM_H

#include <math.h>
#include <Eigen/Core>

inline constexpr float kToleranceF = 1.0e-5F;

inline bool isValidDcm(Eigen::Matrix3f const& dcm) {
    bool valid = true;
    const bool isOrthogonal = (dcm.transpose() * dcm).isApprox(Eigen::Matrix3f::Identity(), kToleranceF);
    const bool detIsValid = fabsf(dcm.determinant() - 1.0F) <= kToleranceF;
    if (!isOrthogonal || !detIsValid) {
        valid = false;
    }
    return valid;
}

#endif  // FP32_XMERA_FSW_ALGORITHMS_VALID_DCM_H
