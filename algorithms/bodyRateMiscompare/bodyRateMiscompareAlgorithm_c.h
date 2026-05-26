#ifndef F32XMERA_BODYRATEMISCOMPAREALGORITHM_C_H
#define F32XMERA_BODYRATEMISCOMPAREALGORITHM_C_H

#include "utilities/plainCAlgorithmDataTypes.h"

#include <stdbool.h>
#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ BodyRateMiscompareAlgorithm instance.
 */
typedef struct BodyRateMiscompareAlgorithm BodyRateMiscompareAlgorithm;

/**
 * @brief POD representation of the BodyRateMiscompareOutput.
 */
typedef struct {
    float omega_BN_B[3];
    bool bodyRateFaultDetected;
} BodyRateMiscompareOutput_c;

/**
 * @brief Construct a new BodyRateMiscompareAlgorithm instance.
 * @return Pointer to a new BodyRateMiscompareAlgorithm (must be destroyed).
 */
BodyRateMiscompareAlgorithm* BodyRateMiscompareAlgorithm_create(void);

/**
 * @brief Destroy a previously created BodyRateMiscompareAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void BodyRateMiscompareAlgorithm_destroy(BodyRateMiscompareAlgorithm* self);

/**
 * @brief Reset the persistence counter to zero.
 * @param self Pointer to the instance.
 */
void BodyRateMiscompareAlgorithm_reset(BodyRateMiscompareAlgorithm* self);

/**
 * @brief Run the update step.
 * @param self      Pointer to the instance.
 * @param imuOmega  IMU body rate vector.
 * @param stOmega   Star tracker body rate vector.
 * @return BodyRateMiscompareOutput_c  The computed output.
 */
BodyRateMiscompareOutput_c BodyRateMiscompareAlgorithm_update(BodyRateMiscompareAlgorithm* self,
                                                              Vector3f_c imuOmega,
                                                              Vector3f_c stOmega);

/**
 * @brief Set the body rate threshold.
 * @param self               Pointer to the instance.
 * @param bodyRateThreshold  Threshold value.
 */
void BodyRateMiscompareAlgorithm_setBodyRateThreshold(BodyRateMiscompareAlgorithm* self, float bodyRateThreshold);

/**
 * @brief Get the current body rate threshold.
 * @param self Pointer to the instance.
 * @return float  The current threshold value.
 */
float BodyRateMiscompareAlgorithm_getBodyRateThreshold(const BodyRateMiscompareAlgorithm* self);

/**
 * @brief Set the fault persistence count.
 * @param self              Pointer to the instance.
 * @param faultPersistenceLimit  Number of consecutive update calls needed to trigger the fault.
 */
void BodyRateMiscompareAlgorithm_setFaultPersistenceLimit(BodyRateMiscompareAlgorithm* self,
                                                          uint32_t faultPersistenceLimit);

/**
 * @brief Get the current fault persistence count.
 * @param self Pointer to the instance.
 * @return uint32_t  The current fault persistence value.
 */
uint32_t BodyRateMiscompareAlgorithm_getFaultPersistenceLimit(const BodyRateMiscompareAlgorithm* self);

/**
 * @brief Set the useImuRates flag.
 * @param self         Pointer to the instance.
 * @param useImuRates  If true, always output IMU rates regardless of miscompare.
 */
void BodyRateMiscompareAlgorithm_setUseImuRates(BodyRateMiscompareAlgorithm* self, bool useImuRates);

/**
 * @brief Get the current useImuRates flag.
 * @param self Pointer to the instance.
 * @return bool  The current useImuRates value.
 */
bool BodyRateMiscompareAlgorithm_getUseImuRates(const BodyRateMiscompareAlgorithm* self);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XIMERA_BODYRATEMISCOMPAREALGORITHM_C_H
