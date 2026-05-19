#ifndef F32XIMERA_EPHEMERIDES_RECENTER_ALGORITHM_C_H
#define F32XIMERA_EPHEMERIDES_RECENTER_ALGORITHM_C_H

#include "ephemeridesRecenterTypes.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ EphemeridesRecenterAlgorithm instance.
 */
typedef struct EphemeridesRecenterAlgorithm EphemeridesRecenterAlgorithm;

/**
 * @brief POD representation of one body's ephemeris payload as used by the
 *        algorithm. The C++ side uses Eigen::Vector3d for position/velocity
 *        and a bool for the moon flag; both are converted in the shim.
 */
typedef struct {
    int bodySpiceId;           /*!< SPICE ID of the body */
    int originalCentralBodyId; /*!< SPICE ID of original central body */
    int isMoon;                /*!< 1 if this body is a moon of another listed body, else 0 */
    double position[3]; /*!< [m] position (input: relative to original central body; output: relative to new central
                           body) */
    double velocity[3]; /*!< [m/s] velocity (input: relative to original central body; output: relative to new central
                           body) */
} BodyEphemerisPayload_c;

/**
 * @brief Bounded array of MAX_NUM_CHANGE_BODIES body payloads. Wrapping the
 *        array in a struct gives the C boundary an unambiguous size and
 *        enables pass-by-value or pass-by-pointer-to-single-instance.
 */
typedef struct {
    BodyEphemerisPayload_c body[MAX_NUM_CHANGE_BODIES];
} BodyEphemerisPayloadArray20_c;

/**
 * @brief Bounded array of MAX_NUM_CHANGE_BODIES SPICE IDs.
 */
typedef struct {
    int id[MAX_NUM_CHANGE_BODIES];
} IntArray20_c;

/**
 * @brief Construct a new EphemeridesRecenterAlgorithm instance.
 * @return Pointer to a new EphemeridesRecenterAlgorithm (must be destroyed).
 */
EphemeridesRecenterAlgorithm* EphemeridesRecenterAlgorithm_create(void);

/**
 * @brief Destroy a previously created EphemeridesRecenterAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void EphemeridesRecenterAlgorithm_destroy(EphemeridesRecenterAlgorithm* self);

/**
 * @brief Validate the configured body topology and pre-compute moon hierarchy.
 *        Throws on invalid configurations (orphan moon, moon-of-moon,
 *        multiple moons per parent).
 * @param self Pointer to the instance.
 */
void EphemeridesRecenterAlgorithm_reset(EphemeridesRecenterAlgorithm* self);

/**
 * @brief Run the recentering update.
 * @param self      Pointer to the instance.
 * @param newBodies Pointer to a single instance containing input position/velocity
 *                  for every configured body (in the order they were added).
 *                  Indices beyond the configured body count are unused.
 * @return BodyEphemerisPayloadArray20_c  Output position/velocity for each body
 *         relative to the new central body.
 */
BodyEphemerisPayloadArray20_c EphemeridesRecenterAlgorithm_updateState(EphemeridesRecenterAlgorithm* self,
                                                                       const BodyEphemerisPayloadArray20_c* newBodies);

/**
 * @brief Set the SPICE ID of the new central body.
 * @param self        Pointer to the instance.
 * @param bodySpiceId SPICE ID of the new central body.
 */
void EphemeridesRecenterAlgorithm_setNewZeroBaseId(EphemeridesRecenterAlgorithm* self, int bodySpiceId);

/**
 * @brief Get the SPICE ID of the new central body.
 * @param self Pointer to the instance.
 * @return int  SPICE ID of the new central body.
 */
int EphemeridesRecenterAlgorithm_getNewZeroBase(const EphemeridesRecenterAlgorithm* self);

/**
 * @brief Set the SPICE ID of the previous common central body.
 * @param self        Pointer to the instance.
 * @param bodySpiceId SPICE ID of the previous common central body.
 */
void EphemeridesRecenterAlgorithm_setPreviousCommonZeroBase(EphemeridesRecenterAlgorithm* self, int bodySpiceId);

/**
 * @brief Get the SPICE ID of the previous common central body.
 * @param self Pointer to the instance.
 * @return int  SPICE ID of the previous common central body.
 */
int EphemeridesRecenterAlgorithm_getPreviousCommonZeroBase(const EphemeridesRecenterAlgorithm* self);

/**
 * @brief Get the number of bodies that have been added to the algorithm.
 * @param self Pointer to the instance.
 * @return uint32_t  The number of configured bodies.
 */
uint32_t EphemeridesRecenterAlgorithm_getNumberOfBodies(const EphemeridesRecenterAlgorithm* self);

/**
 * @brief Get all configured body SPICE IDs.
 * @param self Pointer to the instance.
 * @return IntArray20_c  SPICE IDs of every configured body, in insertion order.
 *         Trailing entries beyond the configured count are zero.
 */
IntArray20_c EphemeridesRecenterAlgorithm_getAllIds(const EphemeridesRecenterAlgorithm* self);

/**
 * @brief Add a body to the recenter list.
 * @param self Pointer to the instance.
 * @param body Pointer to a single BodyToRecenter describing the body.
 */
void EphemeridesRecenterAlgorithm_addBodyEphemerisToRecenter(EphemeridesRecenterAlgorithm* self,
                                                             const BodyToRecenter* body);

/**
 * @brief Remove every body from the recenter list.
 * @param self Pointer to the instance.
 */
void EphemeridesRecenterAlgorithm_clearAllBodies(EphemeridesRecenterAlgorithm* self);

/**
 * @brief Look up the index of a body by its SPICE ID.
 * @param self        Pointer to the instance.
 * @param bodySpiceId SPICE ID to look up.
 * @return uint32_t  The index of the body in the configured list.
 */
uint32_t EphemeridesRecenterAlgorithm_findBodyIndex(const EphemeridesRecenterAlgorithm* self, int bodySpiceId);

/**
 * @brief Get the MAX_NUM_CHANGE_BODIES constant for Ada validation.
 * @return uint32_t  The value of MAX_NUM_CHANGE_BODIES.
 */
uint32_t EphemeridesRecenterAlgorithm_getMaxNumChangeBodies(void);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XIMERA_EPHEMERIDES_RECENTER_ALGORITHM_C_H
