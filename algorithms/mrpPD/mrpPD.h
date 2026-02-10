#ifndef XMERAF32_MRP_PD_H
#define XMERAF32_MRP_PD_H

#include <stdint.h>

#include <Eigen/Dense>

#include "mrpPDAlgorithm.h"
#include "msgPayloadDef/AttGuidMsgF32Payload.h"
#include "msgPayloadDef/CmdTorqueBodyMsgF32Payload.h"
#include "msgPayloadDef/VehicleConfigMsgF32Payload.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>

/*! @brief MRP PD control class. */
class MrpPD : public SysModel {
   public:
    MrpPD() = default;
    ~MrpPD() = default;

    void reset(uint64_t currentSimNanos) override;
    void updateState(uint64_t currentSimNanos) override;
    void setDerivativeGainP(float P);
    float getDerivativeGainP() const;
    void setKnownTorquePntB_B(const Eigen::Vector3f& knownTorquePntB_B);
    const Eigen::Vector3f& getKnownTorquePntB_B() const;
    void setProportionalGainK(float K);
    float getProportionalGainK() const;

    ReadFunctor<AttGuidMsgF32Payload> guidInMsg;             //!< Attitude guidance input message
    ReadFunctor<VehicleConfigMsgF32Payload> vehConfigInMsg;  //!< Vehicle configuration input message
    Message<CmdTorqueBodyMsgF32Payload> cmdTorqueOutMsg;     //!< Commanded torque output message

   private:
    MrpPDAlgorithm algorithm{};  //!< Algorithm for mrpPD control logic (BSK-agnostic)
};

#endif
