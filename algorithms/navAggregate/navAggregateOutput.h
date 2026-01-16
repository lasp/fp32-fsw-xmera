#ifndef F32XIMERA_NAV_AGGREGATE_OUTPUT_H
#define F32XIMERA_NAV_AGGREGATE_OUTPUT_H

#include "msgPayloadDef/NavAttMsgF32Payload.h"
#include "msgPayloadDef/NavTransMsgF32Payload.h"

#ifdef __cplusplus
extern "C" {
#endif

/*! structure containing the attitude and translational navigation out messages */
typedef struct {
    NavAttMsgF32Payload navAttOut;     /*!< attitude navigation out message payload */
    NavTransMsgF32Payload navTransOut; /*!< translation navigation out message payload */
} AggregateOutput;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XIMERA_NAV_AGGREGATE_OUTPUT_H
