/* MIT License
 *
 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#ifndef F32XIMERA_SUNLINEEPHEMALGORITHM_C_H
#define F32XIMERA_SUNLINEEPHEMALGORITHM_C_H

#include "msgPayloadDef/EphemerisMsgF32Payload.h"
#include "msgPayloadDef/NavAttMsgF32Payload.h"
#include "msgPayloadDef/NavTransMsgF32Payload.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ SunlineEphemAlgorithm instance.
 */
typedef struct SunlineEphemAlgorithm SunlineEphemAlgorithm;

/**
 * @brief Construct a new SunlineEphemAlgorithm instance.
 * @return Pointer to a new SunlineEphemAlgorithm (must be destroyed).
 */
SunlineEphemAlgorithm* SunlineEphemAlgorithm_create(void);

/**
 * @brief Destroy a previously created SunlineEphemAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void SunlineEphemAlgorithm_destroy(SunlineEphemAlgorithm* self);

/**
 * @brief Compute ephemeris-based sunline heading in body frame.
 * @param self   Pointer to the instance.
 * @param sunPos Pointer to sun ephemeris message payload.
 * @param scPos  Pointer to spacecraft position message payload.
 * @param scAtt  Pointer to spacecraft attitude message payload.
 * @return NavAttMsgF32Payload  Navigation message containing sunline direction in body frame.
 */
NavAttMsgF32Payload SunlineEphemAlgorithm_updateState(const SunlineEphemAlgorithm* self,
                                                      const EphemerisMsgF32Payload* sunPos,
                                                      const NavTransMsgF32Payload* scPos,
                                                      const NavAttMsgF32Payload* scAtt);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XIMERA_SUNLINEEPHEMALGORITHM_C_H
