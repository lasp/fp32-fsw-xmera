/* MIT License
 *
 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#ifndef F32XIMERA_SUNSEARCHALGORITHM_C_H
#define F32XIMERA_SUNSEARCHALGORITHM_C_H

#include "msgPayloadDef/AttGuidMsgF32Payload.h"
#include "msgPayloadDef/NavAttMsgF32Payload.h"
#include "sunSearchTypes.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ SunSearchAlgorithm instance.
 */
typedef struct SunSearchAlgorithm SunSearchAlgorithm;

/**
 * @brief Construct a new SunSearchAlgorithm instance.
 * @return Pointer to a new SunSearchAlgorithm (must be destroyed).
 */
SunSearchAlgorithm* SunSearchAlgorithm_create(void);

/**
 * @brief Destroy a previously created SunSearchAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void SunSearchAlgorithm_destroy(SunSearchAlgorithm* self);

/**
 * @brief Reset the algorithm state.
 * @param self              Pointer to the instance.
 * @param currentSimNanos   Current simulation time in nanoseconds.
 * @param principleInertias Principle vehicle inertia terms.
 */
void SunSearchAlgorithm_reset(SunSearchAlgorithm* self,
                              uint64_t currentSimNanos,
                              const PrincipleInertias* principleInertias);

/**
 * @brief Run the update step.
 * @param self            Pointer to the instance.
 * @param currentSimNanos Current simulation time in nanoseconds.
 * @param navAttIn        Pointer to navigation attitude message payload.
 * @return AttGuidMsgF32Payload  The computed guidance message.
 */
AttGuidMsgF32Payload SunSearchAlgorithm_update(const SunSearchAlgorithm* self,
                                               uint64_t currentSimNanos,
                                               const NavAttMsgF32Payload* navAttIn);

/**
 * @brief Append a slew maneuver to the list.
 * @param self                Pointer to the instance.
 * @param slewPropertiesInput Pointer to the slew properties to append.
 */
void SunSearchAlgorithm_setSlewProperties(SunSearchAlgorithm* self, const SlewProperties* slewPropertiesInput);

/**
 * @brief Modify an existing slew maneuver at a given index.
 * @param self                Pointer to the instance.
 * @param slewPropertiesInput Pointer to the new slew properties.
 * @param index               Index of the slew maneuver to modify.
 */
void SunSearchAlgorithm_modifySlewProperties(SunSearchAlgorithm* self,
                                             const SlewProperties* slewPropertiesInput,
                                             uint32_t index);

/**
 * @brief Get the properties of a slew maneuver at a given index.
 * @param self  Pointer to the instance.
 * @param index Index of the slew maneuver.
 * @return SlewProperties  The slew properties at the given index.
 */
SlewProperties SunSearchAlgorithm_getSlewProperties(const SunSearchAlgorithm* self, uint32_t index);

/**
 * @brief Get the NUM_SLEWS constant for Ada validation.
 * @return The value of NUM_SLEWS.
 */
uint32_t SunSearchAlgorithm_getNumSlews(void);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XIMERA_SUNSEARCHALGORITHM_C_H
