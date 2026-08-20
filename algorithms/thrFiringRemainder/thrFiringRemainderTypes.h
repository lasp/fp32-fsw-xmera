#ifndef THR_FIRING_REMAINDER_TYPES_H
#define THR_FIRING_REMAINDER_TYPES_H

#include "msgPayloadDef/definitions.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Thrust pulsing regime selection. */
typedef enum {
    THR_FIRING_REMAINDER_ON_PULSING = 0,
    THR_FIRING_REMAINDER_OFF_PULSING = 1
} ThrFiringRemainderPulsingRegime;

/** @brief Plain-old-data mirror of the C++ ThrFiringRemainderThrusterArray. */
typedef struct {
    uint32_t numThrusters;        /*!< [-] number of thrusters on the vehicle */
    float maxThrust[MAX_EFF_CNT]; /*!< [N] per-thruster maximum thrust */
} ThrFiringRemainderThrusterArray_c;

/** @brief Plain-old-data mirror of the C++ ThrFiringControlParameters. */
typedef struct {
    float thrMinFireTime;                          /*!< [s] minimum commandable thruster fire time */
    float controlPeriod;                           /*!< [s] control period over which the force command applies */
    float onTimeSaturationFactor;                  /*!< [-] control-period multiplier applied when on-time saturates */
    ThrFiringRemainderPulsingRegime pulsingRegime; /*!< [-] on-pulsing or off-pulsing */
} ThrFiringRemainderControlParameters_c;

/** @brief Plain-old-data mirror of the C++ ThrFiringRemainderConfig. */
typedef struct {
    ThrFiringRemainderThrusterArray_c thrusterArray;         /*!< [-] per-thruster maximum thrust */
    ThrFiringRemainderControlParameters_c controlParameters; /*!< [-] firing control parameters */
} ThrFiringRemainderConfig_c;

/** @brief Thruster force command input (POD). */
typedef struct {
    float thrForce[MAX_EFF_CNT]; /*!< [N] Thruster force values */
} ThrFiringRemainderForceCmd;

/** @brief Thruster on-time command output (POD). */
typedef struct {
    float onTimeRequest[MAX_EFF_CNT]; /*!< [s] On-time requests */
} ThrFiringRemainderOnTimeCmd;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // THR_FIRING_REMAINDER_TYPES_H
