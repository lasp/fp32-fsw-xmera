#ifndef F32XMERA_SUNLINEEPHEMALGORITHM_C_H
#define F32XMERA_SUNLINEEPHEMALGORITHM_C_H

#include "utilities/fsw/plainCAlgorithmDataTypes.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ SunlineEphemAlgorithm instance.
 */
typedef struct SunlineEphemAlgorithmHandle SunlineEphemAlgorithmHandle;

/**
 * @brief POD representation of a 3-vector (Eigen::Vector3d).
 */
typedef struct {
    double data[3];
} Vector3d_c;

/**
 * @brief Construct a new SunlineEphemAlgorithm instance.
 * @return Pointer to a new SunlineEphemAlgorithm (must be destroyed).
 */
SunlineEphemAlgorithmHandle* SunlineEphemAlgorithm_create(void);

/**
 * @brief Destroy a previously created SunlineEphemAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void SunlineEphemAlgorithm_destroy(SunlineEphemAlgorithmHandle* self);

/**
 * @brief Compute ephemeris-based sunline heading in body frame.
 * @param self    Pointer to the instance.
 * @param sunPos  Sun inertial position r_SN_N [m].
 * @param scPos   Spacecraft inertial position r_BN_N [m].
 * @param sigmaBN Spacecraft attitude MRP (body relative to inertial).
 * @param result  Out: sunline direction (unit vector) in body frame.
 */
void SunlineEphemAlgorithm_update(const SunlineEphemAlgorithmHandle* self,
                                  const Vector3d_c* sunPos,
                                  const Vector3d_c* scPos,
                                  const Vector3f_c* sigmaBN,
                                  Vector3f_c* result);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_SUNLINEEPHEMALGORITHM_C_H
