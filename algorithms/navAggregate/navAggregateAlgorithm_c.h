#ifndef F32XMERA_NAVAGGREGATEALGORITHM_C_H
#define F32XMERA_NAVAGGREGATEALGORITHM_C_H

#include "navAggregateTypes.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ NavAggregateAlgorithm instance.
 */
typedef struct NavAggregateAlgorithmHandle NavAggregateAlgorithmHandle;

/**
 * @brief Get the maximum aggregate navigation message count.
 * @return The maximum message count (MAX_AGG_NAV_MSG).
 */
uint32_t NavAggregateAlgorithm_getMaxAggNavMsg(void);

/**
 * @brief Report whether a configuration would be accepted by create/setConfig.
 * @param config Pointer to the configuration to check.
 * @return true if the configuration is valid. Never throws, so it can guard the
 *         throwing create/setConfig from an invalid configuration.
 * @note The accepted value ranges are defined by NavAggregateConfig::create; this predicate
 *       reports whether a candidate set would be accepted, without throwing.
 */
bool NavAggregateAlgorithm_validateConfig(uint32_t attTimeIdx,
                                          uint32_t attIdx,
                                          uint32_t rateIdx,
                                          uint32_t sunIdx,
                                          uint32_t attMsgCount,
                                          uint32_t transTimeIdx,
                                          uint32_t posIdx,
                                          uint32_t velIdx,
                                          uint32_t dvIdx,
                                          uint32_t transMsgCount);

/**
 * @brief Construct a new NavAggregateAlgorithm instance from the supplied configuration.
 * @param attTimeIdx    [-] index of the message providing the attitude message time.
 * @param attIdx        [-] index of the message providing the inertial MRP.
 * @param rateIdx       [-] index of the message providing the attitude rate.
 * @param sunIdx        [-] index of the message providing the sun-pointing vector.
 * @param attMsgCount   [-] number of attitude messages available as inputs.
 * @param transTimeIdx  [-] index of the message providing the translation message time.
 * @param posIdx        [-] index of the message providing the inertial position.
 * @param velIdx        [-] index of the message providing the inertial velocity.
 * @param dvIdx         [-] index of the message providing the accumulated delta-v.
 * @param transMsgCount [-] number of translation messages available as inputs.
 * @return Pointer to a new NavAggregateAlgorithm (must be destroyed).
 */
NavAggregateAlgorithmHandle* NavAggregateAlgorithm_create(uint32_t attTimeIdx,
                                                          uint32_t attIdx,
                                                          uint32_t rateIdx,
                                                          uint32_t sunIdx,
                                                          uint32_t attMsgCount,
                                                          uint32_t transTimeIdx,
                                                          uint32_t posIdx,
                                                          uint32_t velIdx,
                                                          uint32_t dvIdx,
                                                          uint32_t transMsgCount);

/**
 * @brief Destroy a previously created NavAggregateAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void NavAggregateAlgorithm_destroy(NavAggregateAlgorithmHandle* self);

/**
 * @brief Replace the algorithm's configuration at runtime.
 * @param self   Pointer to the instance.
 * @param attTimeIdx    [-] index of the message providing the attitude message time.
 * @param attIdx        [-] index of the message providing the inertial MRP.
 * @param rateIdx       [-] index of the message providing the attitude rate.
 * @param sunIdx        [-] index of the message providing the sun-pointing vector.
 * @param attMsgCount   [-] number of attitude messages available as inputs.
 * @param transTimeIdx  [-] index of the message providing the translation message time.
 * @param posIdx        [-] index of the message providing the inertial position.
 * @param velIdx        [-] index of the message providing the inertial velocity.
 * @param dvIdx         [-] index of the message providing the accumulated delta-v.
 * @param transMsgCount [-] number of translation messages available as inputs.
 */
void NavAggregateAlgorithm_setConfig(NavAggregateAlgorithmHandle* self,
                                     uint32_t attTimeIdx,
                                     uint32_t attIdx,
                                     uint32_t rateIdx,
                                     uint32_t sunIdx,
                                     uint32_t attMsgCount,
                                     uint32_t transTimeIdx,
                                     uint32_t posIdx,
                                     uint32_t velIdx,
                                     uint32_t dvIdx,
                                     uint32_t transMsgCount);

/**
 * @brief Run the update step.
 * @param self               Pointer to the instance.
 * @param attMsgsPayloads    Attitude navigation message payloads.
 * @param transMsgsPayloads  Translational navigation message payloads.
 * @return AggregateOutput_c The computed output messages.
 */
AggregateOutput_c NavAggregateAlgorithm_update(const NavAggregateAlgorithmHandle* self,
                                               const NavAttMsgF32PayloadArray10_c* attMsgsPayloads,
                                               const NavTransMsgF32PayloadArray10_c* transMsgsPayloads);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_NAVAGGREGATEALGORITHM_C_H
