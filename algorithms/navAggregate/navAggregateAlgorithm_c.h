/* MIT License
 *
 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#ifndef F32XIMERA_NAVAGGREGATEALGORITHM_C_H
#define F32XIMERA_NAVAGGREGATEALGORITHM_C_H

#include "navAggregateOutput.h"
#include <stdint.h>

#define MAX_AGG_NAV_MSG 10

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ NavAggregateAlgorithm instance.
 */
typedef struct NavAggregateAlgorithm NavAggregateAlgorithm;

/**
 * @brief Get the maximum aggregate navigation message count.
 * @return The maximum message count (MAX_AGG_NAV_MSG).
 */
uint32_t NavAggregateAlgorithm_getMaxAggNavMsg(void);

/**
 * @brief Construct a new NavAggregateAlgorithm instance.
 * @return Pointer to a new NavAggregateAlgorithm (must be destroyed).
 */
NavAggregateAlgorithm* NavAggregateAlgorithm_create(void);

/**
 * @brief Destroy a previously created NavAggregateAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void NavAggregateAlgorithm_destroy(NavAggregateAlgorithm* self);

/**
 * @brief Run the update step.
 * @param self               Pointer to the instance.
 * @param attMsgsPayloads    Pointer to array of attitude navigation message payloads.
 * @param transMsgsPayloads  Pointer to array of translational navigation message payloads.
 * @return AggregateOutput   The computed output messages.
 */
AggregateOutput NavAggregateAlgorithm_update(NavAggregateAlgorithm* self,
                                             const NavAttMsgF32Payload* attMsgsPayloads,
                                             const NavTransMsgF32Payload* transMsgsPayloads);

/**
 * @brief Set the attitude time index.
 * @param self Pointer to the instance.
 * @param idx  The new attitude time index to set.
 */
void NavAggregateAlgorithm_setAttTimeIdx(NavAggregateAlgorithm* self, uint32_t idx);

/**
 * @brief Get the current attitude time index.
 * @param self Pointer to the instance.
 * @return uint32_t  The current attitude time index.
 */
uint32_t NavAggregateAlgorithm_getAttTimeIdx(const NavAggregateAlgorithm* self);

/**
 * @brief Set the translation time index.
 * @param self Pointer to the instance.
 * @param idx  The new translation time index to set.
 */
void NavAggregateAlgorithm_setTransTimeIdx(NavAggregateAlgorithm* self, uint32_t idx);

/**
 * @brief Get the current translation time index.
 * @param self Pointer to the instance.
 * @return uint32_t  The current translation time index.
 */
uint32_t NavAggregateAlgorithm_getTransTimeIdx(const NavAggregateAlgorithm* self);

/**
 * @brief Set the attitude index.
 * @param self Pointer to the instance.
 * @param idx  The new attitude index to set.
 */
void NavAggregateAlgorithm_setAttIdx(NavAggregateAlgorithm* self, uint32_t idx);

/**
 * @brief Get the current attitude index.
 * @param self Pointer to the instance.
 * @return uint32_t  The current attitude index.
 */
uint32_t NavAggregateAlgorithm_getAttIdx(const NavAggregateAlgorithm* self);

/**
 * @brief Set the rate index.
 * @param self Pointer to the instance.
 * @param idx  The new rate index to set.
 */
void NavAggregateAlgorithm_setRateIdx(NavAggregateAlgorithm* self, uint32_t idx);

/**
 * @brief Get the current rate index.
 * @param self Pointer to the instance.
 * @return uint32_t  The current rate index.
 */
uint32_t NavAggregateAlgorithm_getRateIdx(const NavAggregateAlgorithm* self);

/**
 * @brief Set the position index.
 * @param self Pointer to the instance.
 * @param idx  The new position index to set.
 */
void NavAggregateAlgorithm_setPosIdx(NavAggregateAlgorithm* self, uint32_t idx);

/**
 * @brief Get the current position index.
 * @param self Pointer to the instance.
 * @return uint32_t  The current position index.
 */
uint32_t NavAggregateAlgorithm_getPosIdx(const NavAggregateAlgorithm* self);

/**
 * @brief Set the velocity index.
 * @param self Pointer to the instance.
 * @param idx  The new velocity index to set.
 */
void NavAggregateAlgorithm_setVelIdx(NavAggregateAlgorithm* self, uint32_t idx);

/**
 * @brief Get the current velocity index.
 * @param self Pointer to the instance.
 * @return uint32_t  The current velocity index.
 */
uint32_t NavAggregateAlgorithm_getVelIdx(const NavAggregateAlgorithm* self);

/**
 * @brief Set the accumulated DV index.
 * @param self Pointer to the instance.
 * @param idx  The new accumulated DV index to set.
 */
void NavAggregateAlgorithm_setDvIdx(NavAggregateAlgorithm* self, uint32_t idx);

/**
 * @brief Get the current accumulated DV index.
 * @param self Pointer to the instance.
 * @return uint32_t  The current accumulated DV index.
 */
uint32_t NavAggregateAlgorithm_getDvIdx(const NavAggregateAlgorithm* self);

/**
 * @brief Set the sun index.
 * @param self Pointer to the instance.
 * @param idx  The new sun index to set.
 */
void NavAggregateAlgorithm_setSunIdx(NavAggregateAlgorithm* self, uint32_t idx);

/**
 * @brief Get the current sun index.
 * @param self Pointer to the instance.
 * @return uint32_t  The current sun index.
 */
uint32_t NavAggregateAlgorithm_getSunIdx(const NavAggregateAlgorithm* self);

/**
 * @brief Set the attitude message count.
 * @param self     Pointer to the instance.
 * @param msgCount The new attitude message count to set.
 */
void NavAggregateAlgorithm_setAttMsgCount(NavAggregateAlgorithm* self, uint32_t msgCount);

/**
 * @brief Get the current attitude message count.
 * @param self Pointer to the instance.
 * @return uint32_t  The current attitude message count.
 */
uint32_t NavAggregateAlgorithm_getAttMsgCount(const NavAggregateAlgorithm* self);

/**
 * @brief Set the translational message count.
 * @param self     Pointer to the instance.
 * @param msgCount The new translational message count to set.
 */
void NavAggregateAlgorithm_setTransMsgCount(NavAggregateAlgorithm* self, uint32_t msgCount);

/**
 * @brief Get the current translational message count.
 * @param self Pointer to the instance.
 * @return uint32_t  The current translational message count.
 */
uint32_t NavAggregateAlgorithm_getTransMsgCount(const NavAggregateAlgorithm* self);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XIMERA_NAVAGGREGATEALGORITHM_C_H
