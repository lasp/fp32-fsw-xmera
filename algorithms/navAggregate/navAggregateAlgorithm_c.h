#ifndef F32XMERA_NAVAGGREGATEALGORITHM_C_H
#define F32XMERA_NAVAGGREGATEALGORITHM_C_H

#include "navAggregateTypes.h"
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
 * @brief Construct a new NavAggregateAlgorithm instance from the supplied configuration.
 * @param config Pointer to the configuration to apply (validated; throws on invalid input).
 * @return Pointer to a new NavAggregateAlgorithm (must be destroyed).
 */
NavAggregateAlgorithmHandle* NavAggregateAlgorithm_create(const NavAggregateConfig_c* config);

/**
 * @brief Destroy a previously created NavAggregateAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void NavAggregateAlgorithm_destroy(NavAggregateAlgorithmHandle* self);

/**
 * @brief Replace the algorithm's configuration at runtime.
 * @param self   Pointer to the instance.
 * @param config Pointer to the configuration to apply (validated; throws on invalid input).
 */
void NavAggregateAlgorithm_setConfig(NavAggregateAlgorithmHandle* self, const NavAggregateConfig_c* config);

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
