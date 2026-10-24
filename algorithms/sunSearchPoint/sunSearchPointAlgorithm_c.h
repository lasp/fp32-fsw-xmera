#ifndef F32XMERA_SUNSEARCHPOINTALGORITHM_C_H
#define F32XMERA_SUNSEARCHPOINTALGORITHM_C_H

#include "sunSearchPointTypes.h"
#include "utilities/fsw/plainCAlgorithmDataTypes.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ SunSearchPointAlgorithm instance.
 */
typedef struct SunSearchPointAlgorithmHandle SunSearchPointAlgorithmHandle;

/**
 * @brief The sun-search rotation sequence, sized so the bound crosses the boundary in the type.
 */
typedef struct {
    RotationProperties_c rotations[SUN_SEARCH_POINT_NUM_ROTATIONS];
} RotationPropertiesArray4_c;

/**
 * @brief C-compatible sun search point attitude guidance output.
 */
typedef struct {
    Vector3f_c sigma_BR;   /*!< attitude error (MRPs) of B relative to R */
    Vector3f_c omega_BR_B; /*!< [rad/s] body rate error of B relative to R in B frame */
    Vector3f_c omega_RN_B; /*!< [rad/s] reference frame rate of R relative to N in B frame */
    bool sunNotFound;      /*!< [-] true once the search fails to acquire the sun (forced to pointing) */
} SunSearchPointOutput_c;

/**
 * @brief Get the sun-search rotation count for Ada validation.
 * @return The number of rotation slots in the sun-search sequence.
 */
uint32_t SunSearchPointAlgorithm_getNumRotations(void);

/**
 * @brief Get the largest accepted observationThreshold for Ada validation.
 * @return kMaxNumCssSensors, the CSS count a threshold can never usefully exceed.
 */
uint32_t SunSearchPointAlgorithm_getMaxObservationThreshold(void);

/**
 * @brief Report whether a configuration would be accepted by create/setConfig.
 * @param rotations            [-] sun-search rotation sequence.
 * @param sHatBdyCmd           [-] commanded body vector to point at the sun.
 * @param sunAxisSpinRate      [rad/s] constant spin rate about the sun heading vector.
 * @param omega_RN_B           [rad/s] fallback body rate when no sun direction is available.
 * @param observationThreshold [-] CSS count at or above which to transition to pointing.
 * @param controlPeriod        [s] per-update time step; advances the search timeline.
 * @param deadbandAngle        [rad] sun angle error below which sigma_BR is zeroed while pointing.
 * @param deadbandRate         [rad/s] rate error norm below which omega_BR_B is zeroed while pointing.
 * @return true if the configuration is valid. Never throws, so it can guard the
 *         throwing create/setConfig from an invalid configuration.
 * @note The accepted value ranges are defined by SunSearchPointConfig::create; this predicate
 *       reports whether a candidate set would be accepted, without throwing.
 */
bool SunSearchPointAlgorithm_validateConfig(const RotationPropertiesArray4_c* rotations,
                                            Vector3f_c sHatBdyCmd,
                                            float sunAxisSpinRate,
                                            Vector3f_c omega_RN_B,
                                            uint32_t observationThreshold,
                                            float controlPeriod,
                                            float deadbandAngle,
                                            float deadbandRate);

/**
 * @brief Construct a new SunSearchPointAlgorithm instance from the supplied configuration.
 * Validate the values with validateConfig before calling; throws on invalid input.
 * @param rotations            [-] sun-search rotation sequence.
 * @param sHatBdyCmd           [-] commanded body vector to point at the sun.
 * @param sunAxisSpinRate      [rad/s] constant spin rate about the sun heading vector.
 * @param omega_RN_B           [rad/s] fallback body rate when no sun direction is available.
 * @param observationThreshold [-] CSS count at or above which to transition to pointing.
 * @param controlPeriod        [s] per-update time step; advances the search timeline.
 * @param deadbandAngle        [rad] sun angle error below which sigma_BR is zeroed while pointing.
 * @param deadbandRate         [rad/s] rate error norm below which omega_BR_B is zeroed while pointing.
 * @return Pointer to a new SunSearchPointAlgorithm (must be destroyed).
 */
SunSearchPointAlgorithmHandle* SunSearchPointAlgorithm_create(const RotationPropertiesArray4_c* rotations,
                                                              Vector3f_c sHatBdyCmd,
                                                              float sunAxisSpinRate,
                                                              Vector3f_c omega_RN_B,
                                                              uint32_t observationThreshold,
                                                              float controlPeriod,
                                                              float deadbandAngle,
                                                              float deadbandRate);

/**
 * @brief Destroy a previously created SunSearchPointAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void SunSearchPointAlgorithm_destroy(SunSearchPointAlgorithmHandle* self);

/**
 * @brief Install a new configuration (parameters only; call _reInitialize to re-arm the search
 *        phase). Validate the values with validateConfig before calling; throws on invalid input.
 * @param self                 Pointer to the instance.
 * @param rotations            [-] sun-search rotation sequence.
 * @param sHatBdyCmd           [-] commanded body vector to point at the sun.
 * @param sunAxisSpinRate      [rad/s] constant spin rate about the sun heading vector.
 * @param omega_RN_B           [rad/s] fallback body rate when no sun direction is available.
 * @param observationThreshold [-] CSS count at or above which to transition to pointing.
 * @param controlPeriod        [s] per-update time step; advances the search timeline.
 * @param deadbandAngle        [rad] sun angle error below which sigma_BR is zeroed while pointing.
 * @param deadbandRate         [rad/s] rate error norm below which omega_BR_B is zeroed while pointing.
 */
void SunSearchPointAlgorithm_setConfig(SunSearchPointAlgorithmHandle* self,
                                       const RotationPropertiesArray4_c* rotations,
                                       Vector3f_c sHatBdyCmd,
                                       float sunAxisSpinRate,
                                       Vector3f_c omega_RN_B,
                                       uint32_t observationThreshold,
                                       float controlPeriod,
                                       float deadbandAngle,
                                       float deadbandRate);

/**
 * @brief Re-arm the runtime state machine so the next update begins a fresh search sequence.
 * @param self Pointer to the instance.
 */
void SunSearchPointAlgorithm_reInitialize(SunSearchPointAlgorithmHandle* self);

/**
 * @brief Run the update step.
 * @param self             Pointer to the instance.
 * @param rHat_SB_B        Sun direction vector in body frame.
 * @param omega_BN_B       Inertial body angular velocity in body frame.
 * @param numCssViewingSun Number of valid coarse-sun-sensor observations this cycle.
 * @return SunSearchPointOutput_c  The computed guidance output.
 */
SunSearchPointOutput_c SunSearchPointAlgorithm_update(SunSearchPointAlgorithmHandle* self,
                                                      Vector3f_c rHat_SB_B,
                                                      Vector3f_c omega_BN_B,
                                                      uint32_t numCssViewingSun);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_SUNSEARCHPOINTALGORITHM_C_H
