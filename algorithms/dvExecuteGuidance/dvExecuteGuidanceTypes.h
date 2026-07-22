#ifndef F32XMERA_DV_EXECUTE_GUIDANCE_TYPES_H
#define F32XMERA_DV_EXECUTE_GUIDANCE_TYPES_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Plain-old-data mirror of the C++ DvExecuteGuidanceOutput fields.
 *  - burnExecuting       [-] flag indicating whether the burn is in progress
 *  - burnComplete        [-] flag indicating whether the burn has completed
 *  - commandThrustersOff [-] caller should write a zeroed thruster on-time command this step
 */
typedef struct {
    uint32_t burnExecuting;
    uint32_t burnComplete;
    bool commandThrustersOff;
} DvExecuteGuidanceOutput_c;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_DV_EXECUTE_GUIDANCE_TYPES_H
