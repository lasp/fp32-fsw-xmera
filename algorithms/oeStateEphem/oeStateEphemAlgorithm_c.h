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
 * @brief Opaque handle to a C++ OEStateEphemConfig instance.
 *
 * The config is a heap-resident staging object built incrementally: create (or reset) to the
 * empty state, setScalars, then addArc once per active arc. Each mutating call validates its
 * input in the C++ layer and this shim converts the throw to a boolean, so the config is at
 * every moment either empty or a valid configuration; the arc count is owned by the config
 * and incremented by addArc. Building arc by arc means no caller ever needs a table-sized
 * buffer on its stack. Once built, the config is handed to the algorithm by handle
 * (create/setConfig below) and can be reset and reused for the next build.
 */
typedef struct OEStateEphemConfigHandle OEStateEphemConfigHandle;

/**
 * @brief Allocate a new configuration in the empty state (zero arcs, zeroed storage).
 * @return The new config, which must be released with OEStateEphemConfig_destroy.
 */
OEStateEphemConfigHandle* OEStateEphemConfig_create(void);

/**
 * @brief Destroy a previously created configuration.
 * @param self Pointer to the config to destroy.
 */
void OEStateEphemConfig_destroy(OEStateEphemConfigHandle* self);

/**
 * @brief Return a configuration to the empty state (zero arcs, zeroed storage) for reuse.
 * @param self Pointer to the config.
 */
void OEStateEphemConfig_reset(OEStateEphemConfigHandle* self);

/**
 * @brief Set the scalar half of a configuration.
 * @param self                              Pointer to the config.
 * @param centralBodyGravitationalParameter [m^3/s^2] central-body gravitational parameter.
 * @param ephemerisTimeJ2000                [s] ephemeris time offset referenced to J2000.
 * @param vehicleTimeOffset                 [s] vehicle clock time offset.
 * @return true on success; false if a value was rejected (the config is unmodified).
 */
bool OEStateEphemConfig_setScalars(OEStateEphemConfigHandle* self,
                                   double centralBodyGravitationalParameter,
                                   double ephemerisTimeJ2000,
                                   double vehicleTimeOffset);

/**
 * @brief Append one active arc to a configuration. The arc count is owned by the config.
 * @param self   Pointer to the config.
 * @param fitArc The arc to append.
 * @return true on success; false if the arc was rejected or the table is already full
 *         (the config is unmodified).
 */
bool OEStateEphemConfig_addArc(OEStateEphemConfigHandle* self, const ChebyshevFitArc_c* fitArc);

/**
 * @brief Report whether a configuration would be accepted by the algorithm.
 * @param self Pointer to the config.
 * @return true if the configuration is valid; false if it is empty (or otherwise invalid).
 *         Because every mutation validates its input, an empty config is the only reachable
 *         invalid state; the full re-check is defense in depth.
 */
bool OEStateEphemConfig_validate(OEStateEphemConfigHandle* self);

/**
 * @brief Construct a new OEStateEphemAlgorithm from a configuration.
 * @param config The configuration to copy into the algorithm. Validate with
 *               OEStateEphemConfig_validate before calling; throws on an invalid config.
 * @return Pointer to a new OEStateEphemAlgorithm (must be destroyed).
 */
OEStateEphemAlgorithmHandle* OEStateEphemAlgorithm_create(OEStateEphemConfigHandle* config);

/**
 * @brief Destroy a previously created OEStateEphemAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void OEStateEphemAlgorithm_destroy(OEStateEphemAlgorithmHandle* self);

/**
 * @brief Replace the algorithm's configuration.
 * @param self   Pointer to the instance.
 * @param config The configuration to copy into the algorithm. Validate with
 *               OEStateEphemConfig_validate before calling; throws on an invalid config,
 *               leaving the active configuration intact.
 */
void OEStateEphemAlgorithm_setConfig(OEStateEphemAlgorithmHandle* self, OEStateEphemConfigHandle* config);

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
