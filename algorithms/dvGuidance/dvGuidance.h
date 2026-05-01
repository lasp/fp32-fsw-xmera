// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef F32XMERA_DV_GUIDANCE_H
#define F32XMERA_DV_GUIDANCE_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/AttRefMsgPayload.h>
#include <architecture/msgPayloadDef/DvBurnCmdMsgPayload.h>

#include <stdint.h>

/*! Generates an attitude reference for a delta-V burn whose direction rotates at a constant rate. */
class DvGuidance : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;
    Message<AttRefMsgPayload> attRefOutMsg;
    ReadFunctor<DvBurnCmdMsgPayload> burnDataInMsg;
};

#endif
