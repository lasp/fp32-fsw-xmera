#ifndef F32XMERA_INERTIAL3D_TYPES_H
#define F32XMERA_INERTIAL3D_TYPES_H

#include "utilities/fsw/plainCAlgorithmDataTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/*! @brief C-compatible mirror of the C++ Inertial3DConfig. */
typedef struct {
    Vector3f_c sigma_RN; /*!< [-] MRP from inertial frame N to reference frame R (must be finite) */
} Inertial3DConfig_c;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_INERTIAL3D_TYPES_H
