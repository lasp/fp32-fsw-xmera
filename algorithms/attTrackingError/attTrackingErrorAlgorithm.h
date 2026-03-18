/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#ifndef F32XIMERA_ATT_TRACKING_ERROR_ALGORITHM_H
#define F32XIMERA_ATT_TRACKING_ERROR_ALGORITHM_H

#include "msgPayloadDef/AttGuidMsgF32Payload.h"
#include "msgPayloadDef/AttRefMsgF32Payload.h"
#include "msgPayloadDef/NavAttMsgF32Payload.h"

#include <Eigen/Core>

class AttTrackingErrorAlgorithm {
   public:
    void reset(uint64_t callTime);  //!< Method for algorithm reset
    AttGuidMsgF32Payload update(AttRefMsgF32Payload& attRefInMsg,
                                NavAttMsgF32Payload& attNavInMsg) const;  //!< Algorithm update method
};

#endif
