#ifndef F32XMERA_SUN_SAFE_POINT_TYPES_H
#define F32XMERA_SUN_SAFE_POINT_TYPES_H

#include <Eigen/Core>

#ifdef __cplusplus
extern "C" {
#endif

/*! @brief Structure containing the sun safe point attitude guidance output */
typedef struct {
    Eigen::Vector3f sigma_BR;    //!< attitude error (MRPs) of B relative to R
    Eigen::Vector3f omega_BR_B;  //!< [rad/s] body rate error of B relative to R in B frame
    Eigen::Vector3f omega_RN_B;  //!< [rad/s] reference frame rate of R relative to N in B frame
} SunSafePointOutput;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif
