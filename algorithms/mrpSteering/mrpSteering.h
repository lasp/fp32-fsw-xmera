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
#include <memory>

/*! @brief MRP steering attitude control adapter. */
class MrpSteering final : public SysModel {
   public:
    MrpSteering() = default;
    ~MrpSteering() override = default;

    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    void reconfigure();
    void reInitialize();

    // Phase 1: public config properties -- set before reset().
    float K1{};                         //!< [rad/s] proportional gain applied to MRP errors
    float K3{};                         //!< [rad/s] cubic gain applied to MRP error in steering saturation function
    float omegaMax{};                   //!< [rad/s] maximum rate command of steering control
    bool ignoreOuterLoopFeedforward{};  //!< [-] whether the outer-loop feedforward term is excluded
    float P{};                          //!< [N*m*s] rate error feedback gain
    float Ki{};                         //!< [N*m] integral feedback gain on the rate error
    float integralLimit{};              //!< [N*m] integral limit to avoid wind-up
    Eigen::Vector3f knownTorquePntB_B = Eigen::Vector3f::Zero();  //!< [N*m] known external torque in body frame
    float controlPeriod{};                                        //!< [s] time between two algorithm update calls

    Message<CmdTorqueBodyMsgF32Payload> cmdTorqueOutMsg;     //!< commanded torque output message
    ReadFunctor<AttGuidMsgF32Payload> guidInMsg;             //!< attitude guidance input message
    ReadFunctor<VehicleConfigMsgF32Payload> vehConfigInMsg;  //!< vehicle configuration input message
    ReadFunctor<RWSpeedMsgF32Payload> rwSpeedsInMsg;         //!< (optional) RW speed input message
    ReadFunctor<RWAvailabilityMsgPayload> rwAvailInMsg;      //!< (optional) RW availability input message
    ReadFunctor<RWArrayConfigMsgF32Payload> rwParamsInMsg;   //!< (optional) RW configuration parameter input message

   private:
    MrpSteeringConfig toConfig();
    std::unique_ptr<MrpSteeringAlgorithm> algorithm = nullptr;
};

#endif
