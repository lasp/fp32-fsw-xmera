#ifndef F32XMERA_MRP_STEERING_H
#define F32XMERA_MRP_STEERING_H

#include "mrpSteeringAlgorithm.h"
#include "msgPayloadDef/AttGuidMsgF32Payload.h"
#include "msgPayloadDef/CmdTorqueBodyMsgF32Payload.h"
#include "msgPayloadDef/RWArrayConfigMsgF32Payload.h"
#include "msgPayloadDef/RWSpeedMsgF32Payload.h"
#include "msgPayloadDef/VehicleConfigMsgF32Payload.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/RWAvailabilityMsgPayload.h>
#include <stdint.h>
#include <Eigen/Core>

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
    void setP(float gain);
    float getP() const;
    void setKi(float gain);
    float getKi() const;
    void setIntegralLimit(float limit);
    float getIntegralLimit() const;
    void setKnownTorquePntB_B(const Eigen::Vector3f& torque);
    Eigen::Vector3f getKnownTorquePntB_B() const;
    void setControlPeriod(float period);
    float getControlPeriod() const;

    Message<CmdTorqueBodyMsgF32Payload> cmdTorqueOutMsg;     //!< commanded torque output message
    ReadFunctor<AttGuidMsgF32Payload> guidInMsg;             //!< attitude guidance input message
    ReadFunctor<VehicleConfigMsgF32Payload> vehConfigInMsg;  //!< vehicle configuration input message
    ReadFunctor<RWSpeedMsgF32Payload> rwSpeedsInMsg;         //!< (optional) RW speed input message
    ReadFunctor<RWAvailabilityMsgPayload> rwAvailInMsg;      //!< (optional) RW availability input message
    ReadFunctor<RWArrayConfigMsgF32Payload> rwParamsInMsg;   //!< (optional) RW configuration parameter input message

   private:
    MrpSteeringAlgorithm algorithm{};
};

#endif
