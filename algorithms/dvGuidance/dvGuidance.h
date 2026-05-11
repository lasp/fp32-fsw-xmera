// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef F32XMERA_DV_GUIDANCE_H
#define F32XMERA_DV_GUIDANCE_H

#include "dvGuidanceAlgorithm.h"
#include "msgPayloadDef/AttRefMsgF32Payload.h"
#include "msgPayloadDef/DvBurnCmdMsgF32Payload.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>

#include <stdint.h>
#include <memory>

/*! @brief Adapter for the delta-V guidance algorithm. */
class DvGuidance final : public SysModel {
   public:
    DvGuidance() = default;
    ~DvGuidance() override = default;

    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    Message<AttRefMsgF32Payload> attRefOutMsg;
    ReadFunctor<DvBurnCmdMsgF32Payload> burnDataInMsg;

   private:
    std::unique_ptr<DvGuidanceAlgorithm> algorithm = nullptr;
};

#endif
