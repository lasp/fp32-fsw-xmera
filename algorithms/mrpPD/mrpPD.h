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

#include <memory>

/*! @brief MRP PD control class. */
class MrpPD : public SysModel {
   public:
    MrpPD() = default;
    ~MrpPD() override = default;

    void reset(uint64_t currentSimNanos) override;
    void updateState(uint64_t currentSimNanos) override;

    float K{};  //!< [rad/s] proportional gain applied to MRP errors
    float P{};  //!< [N*m*s] rate-error feedback (derivative) gain
    Eigen::Vector3f knownTorquePntB_B =
        Eigen::Vector3f::Zero();  //!< [N*m] known external torque in body-frame components

    ReadFunctor<AttGuidMsgF32Payload> guidInMsg;             //!< Attitude guidance input message
    ReadFunctor<VehicleConfigMsgF32Payload> vehConfigInMsg;  //!< Vehicle configuration input message
    Message<CmdTorqueBodyMsgF32Payload> cmdTorqueOutMsg;     //!< Commanded torque output message

   private:
    std::unique_ptr<MrpPDAlgorithm> algorithm = nullptr;
};

#endif
