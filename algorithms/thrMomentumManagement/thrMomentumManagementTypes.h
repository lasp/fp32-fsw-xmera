#ifndef F32XMERA_THR_MOMENTUM_MANAGEMENT_TYPES_H
#define F32XMERA_THR_MOMENTUM_MANAGEMENT_TYPES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Maximum number of reaction wheels handled at the C boundary. Must match RW_EFF_CNT in
   msgPayloadDef/definitions.h (enforced by a static_assert in the adapter). */
#define THR_MOMENTUM_MANAGEMENT_MAX_NUM_RW 36

/**
 * @brief Plain-old-data mirror of the C++ ThrMomentumManagementRwArrayConfiguration.
 *
 *  - numRW must not exceed THR_MOMENTUM_MANAGEMENT_MAX_NUM_RW.
 *  - GsMatrix_B holds the RW spin axes, three components per wheel; each of the first numRW axes
 *    must be a unit vector.
 */
typedef struct {
    uint32_t numRW;                                           /*!< [-]    number of reaction wheels */
    float GsMatrix_B[3 * THR_MOMENTUM_MANAGEMENT_MAX_NUM_RW]; /*!< [-]    RW spin axes, three per wheel */
    float JsList[THR_MOMENTUM_MANAGEMENT_MAX_NUM_RW];         /*!< [kgm2] RW spin-axis inertias */
} ThrMomentumManagementRwArrayConfiguration_c;

/**
 * @brief Plain-old-data carrier for the per-wheel reaction wheel speeds consumed by update().
 */
typedef struct {
    float wheelSpeeds[THR_MOMENTUM_MANAGEMENT_MAX_NUM_RW]; /*!< [r/s] reaction-wheel speeds */
} ThrMomentumManagementWheelSpeeds_c;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif /* F32XMERA_THR_MOMENTUM_MANAGEMENT_TYPES_H */
