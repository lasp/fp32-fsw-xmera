#ifndef F32XMERA_ATTTRACKINGERRORALGORITHM_C_H
#define F32XMERA_ATTTRACKINGERRORALGORITHM_C_H

#include "attTrackingErrorTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ AttTrackingErrorAlgorithm instance.
 */
typedef struct AttTrackingErrorAlgorithmHandle AttTrackingErrorAlgorithmHandle;

/**
 * @brief Construct a new AttTrackingErrorAlgorithm instance.
 * @return Pointer to a new AttTrackingErrorAlgorithm (must be destroyed).
 */
AttTrackingErrorAlgorithmHandle* AttTrackingErrorAlgorithm_create(void);

/**
 * @brief Destroy a previously created AttTrackingErrorAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void AttTrackingErrorAlgorithm_destroy(AttTrackingErrorAlgorithmHandle* self);

/**
 * @brief Run the update step.
 * @param self   Pointer to the instance.
 * @param navIn  C-compatible navigation attitude input.
 * @param refIn  C-compatible reference attitude input.
 * @return AttGuidOutput_c  The computed guidance output.
 */
AttGuidOutput_c AttTrackingErrorAlgorithm_update(AttTrackingErrorAlgorithmHandle* self,
                                                 AttNavInput_c navIn,
                                                 AttRefInput_c refIn);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // ATTTRACKINGERRORALGORITHM_C_H
