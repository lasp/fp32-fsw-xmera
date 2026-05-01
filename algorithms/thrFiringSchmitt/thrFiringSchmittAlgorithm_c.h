#ifndef F32XMERA_THRFIRINGSCHMITTALGORITHM_C_H
#define F32XMERA_THRFIRINGSCHMITTALGORITHM_C_H

#include "thrFiringSchmittTypes.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Maximum number of thrusters supported */
#define THR_FIRING_SCHMITT_MAX_THRUSTER_COUNT 36

/**
 * @brief Opaque handle to the C++ ThrFiringSchmittAlgorithm instance.
 */
typedef struct ThrFiringSchmittAlgorithm ThrFiringSchmittAlgorithm;

/**
 * @brief POD representation of the ON/OFF duty cycle fraction pair.
 */
typedef struct {
    float levelOn;  /*!< [-] ON duty cycle fraction */
    float levelOff; /*!< [-] OFF duty cycle fraction */
} LevelsOnOff_c;

/**
 * @brief POD representation of a single thruster configuration.
 */
typedef struct {
    float rThrust_B[3];    /*!< [m] location of the thruster in the spacecraft */
    float tHatThrust_B[3]; /*!< [-] unit vector of the thrust direction */
    float maxThrust;       /*!< [N] max thrust */
} ThrusterConfig_c;

/**
 * @brief POD representation of a thruster array configuration.
 */
typedef struct {
    uint32_t numThrusters;                                             /*!< [-] number of thrusters */
    ThrusterConfig_c thrusters[THR_FIRING_SCHMITT_MAX_THRUSTER_COUNT]; /*!< thruster configuration array */
} ThrusterArrayConfig_c;

/**
 * @brief POD representation of the thruster force command input.
 */
typedef struct {
    float thrForce[THR_FIRING_SCHMITT_MAX_THRUSTER_COUNT]; /*!< [N] array of thruster force values */
} ThrusterForceCmd_c;

/**
 * @brief POD representation of the thruster on-time command output.
 */
typedef struct {
    float onTimeRequest[THR_FIRING_SCHMITT_MAX_THRUSTER_COUNT]; /*!< [s] array of on-time requests */
} ThrusterOnTimeCmd_c;

/**
 * @brief Get the maximum thruster count constant for Ada validation.
 * @return The value of THR_FIRING_SCHMITT_MAX_THRUSTER_COUNT.
 */
uint32_t ThrFiringSchmittAlgorithm_getMaxThrusterCount(void);

/**
 * @brief Construct a new ThrFiringSchmittAlgorithm instance.
 * @return Pointer to a new ThrFiringSchmittAlgorithm (must be destroyed).
 */
ThrFiringSchmittAlgorithm* ThrFiringSchmittAlgorithm_create(void);

/**
 * @brief Destroy a previously created ThrFiringSchmittAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void ThrFiringSchmittAlgorithm_destroy(ThrFiringSchmittAlgorithm* self);

/**
 * @brief Reset the algorithm state (clears previous-state thruster history).
 * @param self Pointer to the instance.
 */
void ThrFiringSchmittAlgorithm_reset(ThrFiringSchmittAlgorithm* self);

/**
 * @brief Run the update step.
 * @param self             Pointer to the instance.
 * @param thrusterForceCmd Pointer to thruster force command input.
 * @return ThrusterOnTimeCmd_c  The computed thruster on-time command.
 */
ThrusterOnTimeCmd_c ThrFiringSchmittAlgorithm_update(ThrFiringSchmittAlgorithm* self,
                                                     const ThrusterForceCmd_c* thrusterForceCmd);

/**
 * @brief Configure the thruster array (number of thrusters and per-thruster max thrust).
 * @param self           Pointer to the instance.
 * @param thrusterConfig Pointer to thruster array configuration.
 */
void ThrFiringSchmittAlgorithm_setupThrusters(ThrFiringSchmittAlgorithm* self,
                                              const ThrusterArrayConfig_c* thrusterConfig);

/**
 * @brief Get the ON and OFF duty cycle fractions.
 * @param self Pointer to the instance.
 * @return LevelsOnOff_c  Current ON and OFF duty cycle fractions.
 */
LevelsOnOff_c ThrFiringSchmittAlgorithm_getLevelsOnOff(const ThrFiringSchmittAlgorithm* self);

/**
 * @brief Set the ON and OFF duty cycle fractions.
 * @param self     Pointer to the instance.
 * @param levelOn  ON duty cycle fraction in (0.0, 1.0].
 * @param levelOff OFF duty cycle fraction in [0.0, 1.0); must not exceed levelOn.
 */
void ThrFiringSchmittAlgorithm_setLevelsOnOff(ThrFiringSchmittAlgorithm* self, float levelOn, float levelOff);

/**
 * @brief Get the minimum ON time for thrusters.
 * @param self Pointer to the instance.
 * @return float  Minimum ON time [s].
 */
float ThrFiringSchmittAlgorithm_getThrMinFireTime(const ThrFiringSchmittAlgorithm* self);

/**
 * @brief Set the minimum ON time for thrusters.
 * @param self Pointer to the instance.
 * @param time Minimum ON time [s], must be positive.
 */
void ThrFiringSchmittAlgorithm_setThrMinFireTime(ThrFiringSchmittAlgorithm* self, float time);

/**
 * @brief Get the thrust pulsing regime.
 * @param self Pointer to the instance.
 * @return ThrustPulsingRegime  Current pulsing regime (ON_PULSING or OFF_PULSING).
 */
ThrustPulsingRegime ThrFiringSchmittAlgorithm_getThrustPulsingRegime(const ThrFiringSchmittAlgorithm* self);

/**
 * @brief Set the thrust pulsing regime.
 * @param self          Pointer to the instance.
 * @param pulsingRegime Pulsing regime (ON_PULSING or OFF_PULSING).
 */
void ThrFiringSchmittAlgorithm_setThrustPulsingRegime(ThrFiringSchmittAlgorithm* self,
                                                      ThrustPulsingRegime pulsingRegime);

/**
 * @brief Get the control period.
 * @param self Pointer to the instance.
 * @return float  Control period [s].
 */
float ThrFiringSchmittAlgorithm_getControlPeriod(const ThrFiringSchmittAlgorithm* self);

/**
 * @brief Set the control period (time between two algorithm update calls).
 * @param self   Pointer to the instance.
 * @param period Control period [s], must be positive.
 */
void ThrFiringSchmittAlgorithm_setControlPeriod(ThrFiringSchmittAlgorithm* self, float period);

/**
 * @brief Get the on-time saturation factor.
 * @param self Pointer to the instance.
 * @return float  On-time saturation factor.
 */
float ThrFiringSchmittAlgorithm_getOnTimeSaturationFactor(const ThrFiringSchmittAlgorithm* self);

/**
 * @brief Set the on-time saturation factor (applied to control period when on-time saturates).
 * @param self   Pointer to the instance.
 * @param factor Saturation factor, must be >= 1.0.
 */
void ThrFiringSchmittAlgorithm_setOnTimeSaturationFactor(ThrFiringSchmittAlgorithm* self, float factor);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_THRFIRINGSCHMITTALGORITHM_C_H
