#ifndef F32XMERA_ATT_TRACKING_ERROR_ALGORITHM_H
#define F32XMERA_ATT_TRACKING_ERROR_ALGORITHM_H

#include "msgPayloadDef/AttGuidMsgF32Payload.h"
#include "msgPayloadDef/AttRefMsgF32Payload.h"
#include "msgPayloadDef/NavAttMsgF32Payload.h"

#include <Eigen/Core>

class AttTrackingErrorAlgorithm {
   public:
    AttGuidMsgF32Payload update(AttRefMsgF32Payload& attRefInMsg,
                                NavAttMsgF32Payload& attNavInMsg) const;  //!< Algorithm update method
};

#endif
