#ifndef F32XMERA_THR_FIRING_SCHMITT_TYPES_H
#define F32XMERA_THR_FIRING_SCHMITT_TYPES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Maximum number of thrusters supported. Must match kMaxThrusterCount in
    msgPayloadDef/definitions.h (enforced by a static_assert in the C shim). */
#define THR_FIRING_SCHMITT_MAX_THRUSTER_COUNT 36

/** @brief Thrust pulsing regime selection. */
typedef enum { THR_FIRING_SCHMITT_ON_PULSING = 0, THR_FIRING_SCHMITT_OFF_PULSING = 1 } ThrFiringSchmittPulsingRegime;

/** @brief Plain-old-data mirror of the C++ ThrFiringSchmittThrusterArray. */
typedef struct {
    uint32_t numThrusters;                                  /*!< [-] number of thrusters on the vehicle */
    float maxThrust[THR_FIRING_SCHMITT_MAX_THRUSTER_COUNT]; /*!< [N] per-thruster maximum thrust */
} ThrFiringSchmittThrusterArray_c;

/** @brief Plain-old-data mirror of the C++ ThrFiringSchmittControlParameters. */
typedef struct {
    float levelOn;                               /*!< [-] ON duty cycle fraction threshold, in (0, 1] */
    float levelOff;                              /*!< [-] OFF duty cycle fraction threshold, in [0, 1) */
    float thrMinFireTime;                        /*!< [s] minimum commandable thruster fire time */
    float controlPeriod;                         /*!< [s] control period over which the force command applies */
    float onTimeSaturationFactor;                /*!< [-] control-period multiplier applied when on-time saturates */
    ThrFiringSchmittPulsingRegime pulsingRegime; /*!< [-] on-pulsing or off-pulsing */
} ThrFiringSchmittControlParameters_c;

/** @brief Plain-old-data mirror of the C++ ThrFiringSchmittConfig. */
typedef struct {
    ThrFiringSchmittThrusterArray_c thrusterArray;         /*!< [-] per-thruster maximum thrust */
    ThrFiringSchmittControlParameters_c controlParameters; /*!< [-] Schmitt-trigger firing control parameters */
} ThrFiringSchmittConfig_c;

/** @brief Thruster force command input (POD). */
typedef struct {
    float thrForce[THR_FIRING_SCHMITT_MAX_THRUSTER_COUNT]; /*!< [N] Thruster force values */
} ThrFiringSchmittForceCmd;

/** @brief Thruster on-time command output (POD). */
typedef struct {
    float onTimeRequest[THR_FIRING_SCHMITT_MAX_THRUSTER_COUNT]; /*!< [s] On-time requests */
} ThrFiringSchmittOnTimeCmd;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_THR_FIRING_SCHMITT_TYPES_H
