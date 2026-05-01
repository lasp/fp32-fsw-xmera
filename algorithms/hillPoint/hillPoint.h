// SPDX-License-Identifier: ISC
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef F32XMERA_HILL_POINT_H
#define F32XMERA_HILL_POINT_H

#include "hillPointAlgorithm.h"
#include "msgPayloadDef/AttRefMsgF32Payload.h"
#include "msgPayloadDef/EphemerisMsgF32Payload.h"
#include "msgPayloadDef/NavTransMsgF32Payload.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>

/*! @brief Hill Point attitude guidance adapter. */
class HillPoint final : public SysModel {
   public:
    HillPoint() = default;
    ~HillPoint() override = default;

    void reset(uint64_t currentSimNanos) override;
    void updateState(uint64_t currentSimNanos) override;

    ReadFunctor<NavTransMsgF32Payload> transNavInMsg;
    ReadFunctor<EphemerisMsgF32Payload> celBodyInMsg;
    Message<AttRefMsgF32Payload> attRefOutMsg;

   private:
    HillPointAlgorithm algorithm{HillPointConfig::create()};
    bool planetMsgIsLinked{};
};

#endif
