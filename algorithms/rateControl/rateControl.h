/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#ifndef F32XIMERA_RATE_CONTROL_H
#define F32XIMERA_RATE_CONTROL_H

#include "rateControlAlgorithm.h"

#include "msgPayloadDef/AttGuidMsgF32Payload.h"
#include "msgPayloadDef/CmdTorqueBodyMsgF32Payload.h"
#include "msgPayloadDef/VehicleConfigMsgF32Payload.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>

#include <Eigen/Core>

class RateControl : public SysModel {
   public:
    RateControl() = default;
    ~RateControl() = default;

    void reset(uint64_t currentSimNanos) override;
    void updateState(uint64_t currentSimNanos) override;
    void setDerivativeGainP(float P);
    float getDerivativeGainP() const;
    void setKnownTorquePntB_B(const Eigen::Vector3f& knownTorquePntB_B);
    const Eigen::Vector3f& getKnownTorquePntB_B() const;

    ReadFunctor<AttGuidMsgF32Payload> guidInMsg;             //!< Attitude guidance input message
    ReadFunctor<VehicleConfigMsgF32Payload> vehConfigInMsg;  //!< Vehicle configuration input message
    Message<CmdTorqueBodyMsgF32Payload> cmdTorqueOutMsg;     //!< Commanded torque output message

   private:
    RateControlAlgorithm algorithm{};
};

#endif
