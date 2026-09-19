#ifndef F32XMERA_MOMENTUM_MANAGEMENT_TYPES_H
#define F32XMERA_MOMENTUM_MANAGEMENT_TYPES_H

#include "msgPayloadDef/definitions.h"
#include "utilities/fsw/deviceAvailability.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Plain-old-data mirror of the C++ MomentumManagementRwArrayConfiguration.
 *
 *  - GsMatrix_B holds the RW spin axes, three components per wheel; every axis must be a unit
 *    vector.
 *  - wheelAvailability marks each wheel AVAILABLE or UNAVAILABLE; an unavailable wheel contributes
 *    no momentum.
 */
typedef struct {
    float GsMatrix_B[3 * RW_EFF_CNT];                   /*!< [-]    RW spin axes, three per wheel */
    float JsList[RW_EFF_CNT];                           /*!< [kgm2] RW spin-axis inertias */
    DeviceAvailability_c wheelAvailability[RW_EFF_CNT]; /*!< [-] AVAILABLE / UNAVAILABLE state of each wheel */
} MomentumManagementRwArrayConfiguration_c;

/**
 * @brief Plain-old-data carrier for the per-wheel reaction wheel speeds consumed by update().
 */
typedef struct {
    float wheelSpeeds[RW_EFF_CNT]; /*!< [r/s] reaction-wheel speeds */
} MomentumManagementWheelSpeeds_c;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif /* F32XMERA_MOMENTUM_MANAGEMENT_TYPES_H */
