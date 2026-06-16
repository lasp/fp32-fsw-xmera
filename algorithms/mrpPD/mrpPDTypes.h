#ifndef F32XMERA_MRP_PD_TYPES_H
#define F32XMERA_MRP_PD_TYPES_H

#include "utilities/fsw/plainCAlgorithmDataTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/*! @brief Plain-old-data mirror of the C++ MrpPDConfig. */
typedef struct {
    float K;                      /*!< [rad/s] proportional gain applied to MRP errors (>= 0) */
    float P;                      /*!< [N*m*s] rate-error feedback (derivative) gain (>= 0) */
    Vector3f_c knownTorquePntB_B; /*!< [N*m] known external torque in body-frame components (finite) */
    Matrix3f_c ISCPntB_B;         /*!< [kg*m^2] spacecraft inertia about point B (valid inertia matrix) */
} MrpPDConfig_c;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_MRP_PD_TYPES_H
