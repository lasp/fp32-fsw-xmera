#ifndef F32XMERA_ATTTRACKINGERRORALGORITHM_C_H
#define F32XMERA_ATTTRACKINGERRORALGORITHM_C_H

#include "utilities/plainCAlgorithmDataTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ AttTrackingErrorAlgorithm instance.
 */
typedef struct AttTrackingErrorAlgorithmHandle AttTrackingErrorAlgorithmHandle;

/**
 * @brief C-compatible navigation attitude input.
 */
typedef struct {
    Vector3f_c sigma_BN;
    Vector3f_c omega_BN_B;
} AttNavInput_c;

/**
 * @brief C-compatible reference attitude input.
 */
typedef struct {
    Vector3f_c sigma_RN;
    Vector3f_c omega_RN_N;
    Vector3f_c domega_RN_N;
} AttRefInput_c;

/**
 * @brief C-compatible attitude guidance output.
 */
typedef struct {
    Vector3f_c sigma_BR;
    Vector3f_c omega_BR_B;
    Vector3f_c omega_RN_B;
    Vector3f_c domega_RN_B;
} AttGuidOutput_c;

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
