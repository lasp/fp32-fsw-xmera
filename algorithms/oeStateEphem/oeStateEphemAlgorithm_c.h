#ifndef F32XMERA_OE_STATE_EPHEM_ALGORITHM_C_H
#define F32XMERA_OE_STATE_EPHEM_ALGORITHM_C_H

#include "oeStateEphemTypes.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ OEStateEphemAlgorithm instance.
 */
typedef struct OEStateEphemAlgorithmHandle OEStateEphemAlgorithmHandle;

/**
 * @brief Construct a new OEStateEphemAlgorithm from a validated configuration.
 * @param centralBodyGravitationalParameter [m^3/s^2] central-body gravitational parameter.
 * @param numberOfArcs                      [-] number of populated arcs.
 * @param ephemerisTimeJ2000               [s] ephemeris time offset referenced to J2000.
 * @param vehicleTimeOffset                [s] vehicle clock time offset.
 * @param fitCoefficients                  Table of MAX_OE_RECORDS Chebyshev fit arcs.
 * @return Pointer to a new OEStateEphemAlgorithm (must be destroyed). Validated; throws on invalid input.
 */
OEStateEphemAlgorithmHandle* OEStateEphemAlgorithm_create(double centralBodyGravitationalParameter,
                                                          unsigned int numberOfArcs,
                                                          double ephemerisTimeJ2000,
                                                          double vehicleTimeOffset,
                                                          ChebyshevFitArc_c fitCoefficients[MAX_OE_RECORDS]);

/**
 * @brief Destroy a previously created OEStateEphemAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void OEStateEphemAlgorithm_destroy(OEStateEphemAlgorithmHandle* self);

/**
 * @brief Replace the algorithm's configuration.
 * @param self                             Pointer to the instance.
 * @param centralBodyGravitationalParameter [m^3/s^2] central-body gravitational parameter.
 * @param numberOfArcs                      [-] number of populated arcs.
 * @param ephemerisTimeJ2000               [s] ephemeris time offset referenced to J2000.
 * @param vehicleTimeOffset                [s] vehicle clock time offset.
 * @param fitCoefficients                  Table of MAX_OE_RECORDS Chebyshev fit arcs.
 * Validated; throws on invalid input.
 */
void OEStateEphemAlgorithm_setConfig(OEStateEphemAlgorithmHandle* self,
                                     double centralBodyGravitationalParameter,
                                     unsigned int numberOfArcs,
                                     double ephemerisTimeJ2000,
                                     double vehicleTimeOffset,
                                     ChebyshevFitArc_c fitCoefficients[MAX_OE_RECORDS]);

/**
 * @brief Run the update step to compute Cartesian state from ephemeris.
 * @param self     Pointer to the instance.
 * @param callTime Clock time in nanoseconds.
 * @return CartesianState_c  The computed position and velocity vectors.
 */
CartesianState_c OEStateEphemAlgorithm_update(OEStateEphemAlgorithmHandle* self, uint64_t callTime);

/**
 * @brief Get the MAX_OE_COEFF constant for Ada validation.
 * @return The value of MAX_OE_COEFF.
 */
uint32_t OEStateEphemAlgorithm_getMaxOeCoeff(void);

/**
 * @brief Get the MAX_OE_RECORDS constant for Ada validation.
 * @return The value of MAX_OE_RECORDS.
 */
uint32_t OEStateEphemAlgorithm_getMaxOeRecords(void);

/**
 * @brief Get sizeof(ChebyshevFitArc_c) in bits for Ada ABI validation of the
 *        packed-record arc type crossing the FFI boundary.
 * @return The size of one ChebyshevFitArc_c, in bits.
 */
uint32_t OEStateEphemAlgorithm_getFitArcSizeBits(void);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_OE_STATE_EPHEM_ALGORITHM_C_H
