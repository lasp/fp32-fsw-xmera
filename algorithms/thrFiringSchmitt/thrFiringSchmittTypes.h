#ifndef F32XMERA_THR_FIRING_SCHMITT_TYPES_H
#define F32XMERA_THR_FIRING_SCHMITT_TYPES_H

#include "msgPayloadDef/definitions.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Thrust pulsing regime selection. */
typedef enum { THR_FIRING_SCHMITT_ON_PULSING = 0, THR_FIRING_SCHMITT_OFF_PULSING = 1 } ThrFiringSchmittPulsingRegime;

/** @brief Thruster force command input (POD). */
typedef struct {
    float thrForce[MAX_EFF_CNT]; /*!< [N] Thruster force values */
} ThrFiringSchmittForceCmd;

/** @brief Thruster on-time command output (POD). */
typedef struct {
    float onTimeRequest[MAX_EFF_CNT]; /*!< [s] On-time requests */
} ThrFiringSchmittOnTimeCmd;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_THR_FIRING_SCHMITT_TYPES_H
