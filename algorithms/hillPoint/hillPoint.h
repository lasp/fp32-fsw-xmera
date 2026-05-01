// SPDX-License-Identifier: ISC
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef F32XMERA_HILL_POINT_H
#define F32XMERA_HILL_POINT_H

#include "hillPointAlgorithm.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/AttRefMsgPayload.h>
#include <architecture/msgPayloadDef/EphemerisMsgPayload.h>
#include <architecture/msgPayloadDef/NavTransMsgPayload.h>

/*! @brief Hill Point attitude guidance adapter. */
class HillPoint final : public SysModel {
   public:
    HillPoint() = default;
    ~HillPoint() override = default;

    void reset(uint64_t currentSimNanos) override;
    void updateState(uint64_t currentSimNanos) override;

    ReadFunctor<NavTransMsgPayload> transNavInMsg;
    ReadFunctor<EphemerisMsgPayload> celBodyInMsg;
    Message<AttRefMsgPayload> attRefOutMsg;

   private:
    HillPointAlgorithm algorithm{};
    bool planetMsgIsLinked{};
};

#endif
