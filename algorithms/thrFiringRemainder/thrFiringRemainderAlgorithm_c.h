#ifndef F32XMERA_THRFIRINGREMAINDERALGORITHM_C_H
#define F32XMERA_THRFIRINGREMAINDERALGORITHM_C_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Maximum number of thrusters supported. */
#define THR_FIRING_REMAINDER_MAX_THRUSTER_COUNT 36

/** @brief Thrust pulsing regime selection. */
typedef enum {
    THR_FIRING_REMAINDER_ON_PULSING = 0,
    THR_FIRING_REMAINDER_OFF_PULSING = 1
} ThrFiringRemainderPulsingRegime;

/** @brief Single thruster configuration (POD). */
typedef struct {
    float rThrust_B[3];    /*!< [m] Location of thruster in spacecraft frame */
    float tHatThrust_B[3]; /*!< [-] Unit vector of thrust direction */
    float maxThrust;       /*!< [N] Maximum thrust */
} ThrFiringRemainderThrusterConfig;

/** @brief Thruster array configuration (POD). */
typedef struct {
    uint32_t numThrusters; /*!< [-] Number of thrusters */
    ThrFiringRemainderThrusterConfig thrusters[THR_FIRING_REMAINDER_MAX_THRUSTER_COUNT];
} ThrFiringRemainderArrayConfig;

/** @brief Thruster force command input (POD). */
typedef struct {
    float thrForce[THR_FIRING_REMAINDER_MAX_THRUSTER_COUNT]; /*!< [N] Thruster force values */
} ThrFiringRemainderForceCmd;

/** @brief Thruster on-time command output (POD). */
typedef struct {
    float onTimeRequest[THR_FIRING_REMAINDER_MAX_THRUSTER_COUNT]; /*!< [s] On-time requests */
} ThrFiringRemainderOnTimeCmd;

/** @brief Opaque handle to the C++ ThrFiringRemainderAlgorithm instance. */
typedef struct ThrFiringRemainderAlgorithmHandle ThrFiringRemainderAlgorithmHandle;

/**
 * @brief Get the maximum thruster count constant for validation.
 * @return The maximum thruster count (THR_FIRING_REMAINDER_MAX_THRUSTER_COUNT).
 */
uint32_t ThrFiringRemainderAlgorithm_getMaxThrusterCount(void);

/**
 * @brief Construct a new ThrFiringRemainderAlgorithm instance.
 * @return Pointer to a new ThrFiringRemainderAlgorithm (must be destroyed).
 */
ThrFiringRemainderAlgorithmHandle* ThrFiringRemainderAlgorithm_create(void);

/**
 * @brief Destroy a previously created ThrFiringRemainderAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void ThrFiringRemainderAlgorithm_destroy(ThrFiringRemainderAlgorithmHandle* self);

/**
 * @brief Reset the algorithm state.
 * @param self Pointer to the instance.
 */
void ThrFiringRemainderAlgorithm_reset(ThrFiringRemainderAlgorithmHandle* self);

/**
 * @brief Run the update step.
 * @param self     Pointer to the instance.
 * @param forceCmd Pointer to thruster force command input.
 * @return ThrFiringRemainderOnTimeCmd  The computed on-time command.
 */
ThrFiringRemainderOnTimeCmd ThrFiringRemainderAlgorithm_update(ThrFiringRemainderAlgorithmHandle* self,
                                                               const ThrFiringRemainderForceCmd* forceCmd);

/**
 * @brief Set the thruster array configuration.
 * @param self   Pointer to the instance.
 * @param config Pointer to thruster array configuration.
 */
void ThrFiringRemainderAlgorithm_setThrusters(ThrFiringRemainderAlgorithmHandle* self,
                                              const ThrFiringRemainderArrayConfig* config);

/**
 * @brief Set the minimum thruster fire time.
 * @param self        Pointer to the instance.
 * @param minFireTime Minimum fire time in seconds.
 */
void ThrFiringRemainderAlgorithm_setThrMinFireTime(ThrFiringRemainderAlgorithmHandle* self, float minFireTime);

/**
 * @brief Get the minimum thruster fire time.
 * @param self Pointer to the instance.
 * @return float  Minimum fire time in seconds.
 */
float ThrFiringRemainderAlgorithm_getThrMinFireTime(const ThrFiringRemainderAlgorithmHandle* self);

/**
 * @brief Set the thrust pulsing regime.
 * @param self          Pointer to the instance.
 * @param pulsingRegime The pulsing regime (on-pulsing or off-pulsing).
 */
void ThrFiringRemainderAlgorithm_setThrustPulsingRegime(ThrFiringRemainderAlgorithmHandle* self,
                                                        ThrFiringRemainderPulsingRegime pulsingRegime);

/**
 * @brief Get the thrust pulsing regime.
 * @param self Pointer to the instance.
 * @return ThrFiringRemainderPulsingRegime  The current pulsing regime.
 */
ThrFiringRemainderPulsingRegime ThrFiringRemainderAlgorithm_getThrustPulsingRegime(
    const ThrFiringRemainderAlgorithmHandle* self);

/**
 * @brief Set the control period.
 * @param self   Pointer to the instance.
 * @param period Control period in seconds.
 */
void ThrFiringRemainderAlgorithm_setControlPeriod(ThrFiringRemainderAlgorithmHandle* self, float period);

/**
 * @brief Get the control period.
 * @param self Pointer to the instance.
 * @return float  Control period in seconds.
 */
float ThrFiringRemainderAlgorithm_getControlPeriod(const ThrFiringRemainderAlgorithmHandle* self);

/**
 * @brief Set the on-time saturation factor.
 * @param self   Pointer to the instance.
 * @param factor Saturation factor.
 */
void ThrFiringRemainderAlgorithm_setOnTimeSaturationFactor(ThrFiringRemainderAlgorithmHandle* self, float factor);

/**
 * @brief Get the on-time saturation factor.
 * @param self Pointer to the instance.
 * @return float  The saturation factor.
 */
float ThrFiringRemainderAlgorithm_getOnTimeSaturationFactor(const ThrFiringRemainderAlgorithmHandle* self);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_THRFIRINGREMAINDERALGORITHM_C_H
