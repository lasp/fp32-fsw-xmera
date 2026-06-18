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
 * @brief Plain-old-data mirror of the C++ NavAggregateAttSelection.
 */
typedef struct {
    uint32_t attTimeIdx;  /*!< [-] index of the message providing the attitude message time */
    uint32_t attIdx;      /*!< [-] index of the message providing the inertial MRP */
    uint32_t rateIdx;     /*!< [-] index of the message providing the attitude rate */
    uint32_t sunIdx;      /*!< [-] index of the message providing the sun-pointing vector */
    uint32_t attMsgCount; /*!< [-] number of attitude messages available as inputs */
} NavAggregateAttSelection_c;

/**
 * @brief Plain-old-data mirror of the C++ NavAggregateTransSelection.
 */
typedef struct {
    uint32_t transTimeIdx;  /*!< [-] index of the message providing the translation message time */
    uint32_t posIdx;        /*!< [-] index of the message providing the inertial position */
    uint32_t velIdx;        /*!< [-] index of the message providing the inertial velocity */
    uint32_t dvIdx;         /*!< [-] index of the message providing the accumulated DV */
    uint32_t transMsgCount; /*!< [-] number of translation messages available as inputs */
} NavAggregateTransSelection_c;

/**
 * @brief Plain-old-data mirror of the C++ NavAggregateConfig.
 *
 * Every selection index must be less than MAX_AGG_NAV_MSG_C and each message count must not exceed it; the
 * C++ side validates this via NavAggregateConfig::create and throws on invalid input.
 */
typedef struct {
    NavAggregateAttSelection_c attSelection;     /*!< [-] attitude message selection indices */
    NavAggregateTransSelection_c transSelection; /*!< [-] translation message selection indices */
} NavAggregateConfig_c;

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
