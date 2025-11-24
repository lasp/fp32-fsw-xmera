/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#ifndef F32XMERA_MRP_STEERING_H
#define F32XMERA_MRP_STEERING_H

#include "mrpSteeringAlgorithm.h"
#include "msgPayloadDef/AttGuidMsgF32Payload.h"
#include "msgPayloadDef/RateCmdMsgF32Payload.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <stdint.h>

/*! @brief Data structure for the MRP feedback attitude control routine. */
class MrpSteering final : public SysModel {
   public:
    MrpSteering() = default;
    ~MrpSteering() override = default;

    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    void setK1(float gain);
    float getK1() const;
    void setK3(float gain);
    float getK3() const;
    void setOmegaMax(float omega);
    float getOmegaMax() const;
    void setIgnoreFeedforward(bool ignore);
    bool getIgnoreFeedforward() const;

    Message<RateCmdMsgF32Payload> rateCmdOutMsg;  //!< rate command output message
    ReadFunctor<AttGuidMsgF32Payload> guidInMsg;  //!< attitude guidance input message

   private:
    MrpSteeringAlgorithm algorithm{};
};

#endif
