#ifndef F32XIMERA_EPHEMNAVCONVERTERALGORITHM_C_H
#define F32XIMERA_EPHEMNAVCONVERTERALGORITHM_C_H

#include "ephemNavConverterTypes.h"
#include <stdint.h>

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
 * @param ephemerisInput Pointer to the position and velocity ephemeris inputs.
 * @return OutputNavTransData  The computed translational navigation output.
 */
OutputNavTransData EphemNavConverterAlgorithm_update(EphemNavConverterAlgorithm* self,
                                                     const InputEphemerisData* ephemerisInput);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XIMERA_EPHEMNAVCONVERTERALGORITHM_C_H
