/* MIT License
 *
 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#ifndef F32XIMERA_EPHEMNAVCONVERTERALGORITHM_C_H
#define F32XIMERA_EPHEMNAVCONVERTERALGORITHM_C_H

#include <stdint.h>
#include "msgPayloadDef/EphemerisMsgF32Payload.h"
#include "msgPayloadDef/NavTransMsgF32Payload.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ EphemNavConverterAlgorithm instance.
 */
typedef struct EphemNavConverterAlgorithm EphemNavConverterAlgorithm;

/**
 * @brief Construct a new EphemNavConverterAlgorithm instance.
 * @return Pointer to a new EphemNavConverterAlgorithm (must be destroyed).
 */
EphemNavConverterAlgorithm* EphemNavConverterAlgorithm_create(void);

/**
 * @brief Destroy a previously created EphemNavConverterAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void EphemNavConverterAlgorithm_destroy(EphemNavConverterAlgorithm* self);

/**
 * @brief Run the update step.
 * @param self           Pointer to the instance.
 * @param callTime       Time stamp for update.
 * @param ephemerisInMsg Pointer to ephemeris message payload.
 * @return NavTransMsgF32Payload  The computed navigation translation message.
 */
NavTransMsgF32Payload
EphemNavConverterAlgorithm_update(EphemNavConverterAlgorithm* self,
                                  uint64_t callTime,
                                  const EphemerisMsgF32Payload* ephemerisInMsg);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XIMERA_EPHEMNAVCONVERTERALGORITHM_C_H
