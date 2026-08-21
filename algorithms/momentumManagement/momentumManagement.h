#ifndef F32XMERA_MOMENTUM_MANAGEMENT_H
#define F32XMERA_MOMENTUM_MANAGEMENT_H

#include "momentumManagementAlgorithm.h"
#include <stdint.h>

#include <memory>

#include "msgPayloadDef/CmdTorqueBodyMsgF32Payload.h"
#include "msgPayloadDef/RWArrayConfigMsgF32Payload.h"
#include "msgPayloadDef/RWSpeedMsgF32Payload.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>

/*! @brief Assesses the net reaction wheel momentum and requests the torque needed to dump its excess. */
class MomentumManagement : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    //! Re-validate the module properties and push them onto the live algorithm, leaving its state untouched.
    void reconfigure();

    //! Re-seed the algorithm's runtime integrator state; a pass-through to the algorithm's reInitialize().
    void reInitialize();

    /* declare module public variables */
    float hsMin{};          //!< [Nms]  minimum RW cluster momentum for dumping
    float K{};              //!< [1/s]  proportional gain on the excess momentum (must be > 0)
    float Ki{};             //!< [1/s2] integral gain on the accumulated excess momentum (0 disables it)
    float integralLimit{};  //!< [Nms2] anti-windup clamp on each integral component (must be > 0 if Ki > 0)
    float controlPeriod{};  //!< [s]    integration step between updates (must be > 0 if Ki > 0)

    /* declare module IO interfaces */
    Message<CmdTorqueBodyMsgF32Payload> cmdTorqueOutMsg;        //!< [Nm] requested body-frame dumping torque
    ReadFunctor<RWSpeedMsgF32Payload> rwSpeedsInMsg;            //!< [r/s] reaction wheel speeds input message
    ReadFunctor<RWArrayConfigMsgF32Payload> rwConfigDataInMsg;  //!< [-] RW array configuration input message

   private:
    MomentumManagementConfig toConfig();
    std::unique_ptr<MomentumManagementAlgorithm> algorithm = nullptr;
};

#endif
