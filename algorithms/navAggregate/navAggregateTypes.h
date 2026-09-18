#ifndef F32XMERA_NAV_AGGREGATE_TYPES_H
#define F32XMERA_NAV_AGGREGATE_TYPES_H

#include "msgPayloadDef/NavAttMsgF32Payload.h"
#include "msgPayloadDef/NavTransMsgF32Payload.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Maximum number of aggregate navigation messages handled at the C boundary. Must match MAX_AGG_NAV_MSG in
   navAggregateAlgorithm.h (enforced by a static_assert in the C shim). */
#define MAX_AGG_NAV_MSG_C 10

/**
 * @brief Sized array of attitude navigation message payloads.
 */
typedef struct {
    NavAttMsgF32Payload msg[MAX_AGG_NAV_MSG_C];
} NavAttMsgF32PayloadArray10_c;

/**
 * @brief Sized array of translational navigation message payloads.
 */
typedef struct {
    NavTransMsgF32Payload msg[MAX_AGG_NAV_MSG_C];
} NavTransMsgF32PayloadArray10_c;

/**
 * @brief C-compatible aggregate output containing attitude and translational navigation results.
 */
typedef struct {
    NavAttMsgF32Payload navAttOut;     /*!< attitude navigation output */
    NavTransMsgF32Payload navTransOut; /*!< translation navigation output */
} AggregateOutput_c;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_NAV_AGGREGATE_TYPES_H
