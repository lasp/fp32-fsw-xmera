#ifndef F32XMERA_HILL_POINT_H
#define F32XMERA_HILL_POINT_H

#include "hillPointAlgorithm.h"
#include "msgPayloadDef/AttRefMsgF32Payload.h"
#include "msgPayloadDef/EphemerisMsgF32Payload.h"
#include "msgPayloadDef/NavTransMsgF32Payload.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <memory>

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
    std::unique_ptr<HillPointAlgorithm> algorithm = nullptr;
    bool planetMsgIsLinked{};
};

#endif
