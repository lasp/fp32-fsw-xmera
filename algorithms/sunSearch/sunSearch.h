#ifndef F32XMERA_SUN_SEARCH_H
#define F32XMERA_SUN_SEARCH_H

#include "msgPayloadDef/AttGuidMsgF32Payload.h"
#include "msgPayloadDef/NavAttMsgF32Payload.h"
#include "sunSearchAlgorithm.h"

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <array>
#include <cstdint>
#include <memory>

class SunSearch : public SysModel {
   public:
    void reset(uint64_t callTime) final;
    void updateState(uint64_t callTime) final;

    void reconfigure() const;
    void setRotation(uint32_t index, const RotationProperties& rotation);
    RotationProperties getRotation(uint32_t index) const;

    ReadFunctor<NavAttMsgF32Payload> attNavInMsg;  //!< input msg measured attitude
    Message<AttGuidMsgF32Payload> attGuidOutMsg;   //!< Attitude reference output message

   private:
    SunSearchConfig toConfig() const;
    std::unique_ptr<SunSearchAlgorithm> algorithm = nullptr;
    std::array<RotationProperties, kNumRotations> rotations{};
};

#endif
