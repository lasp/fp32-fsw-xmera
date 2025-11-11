/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#ifndef F32XIMERA_RW_NULL_SPACE_H
#define F32XIMERA_RW_NULL_SPACE_H

#include "msgPayloadDef/RWConstellationMsgF32Payload.h"
#include "msgPayloadDef/RWSpeedMsgF32Payload.h"
#include "msgPayloadDef/RwMotorTorqueMsgF32Payload.h"
#include "rwNullSpaceAlgorithm.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>

#include <stdint.h>

/*! @brief The configuration structure for the rwNullSpace module.  */
class RwNullSpace : public SysModel {
   public:
    RwNullSpace() = default;
    ~RwNullSpace() final = default;

    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    void setOmegaGain(float gain);
    float getOmegaGain() const;

    ReadFunctor<RwMotorTorqueMsgF32Payload> rwMotorTorqueInMsg;  //!< [-] The name of the Input message
    ReadFunctor<RWSpeedMsgF32Payload> rwSpeedsInMsg;             //!< [-] The name of the input RW speeds
    ReadFunctor<RWSpeedMsgF32Payload> rwDesiredSpeedsInMsg;      //!< [-] (optional) The name of the desired RW speeds
    ReadFunctor<RWConstellationMsgF32Payload> rwConfigInMsg;     //!< [-] The name of the RWA configuration message
    Message<RwMotorTorqueMsgF32Payload> rwMotorTorqueOutMsg;     //!< [-] The name of the output message

   private:
    RwNullSpaceAlgorithm algorithm{};
};

#endif
