#ifndef F32XMERA_MRP_ROTATION_H
#define F32XMERA_MRP_ROTATION_H

#include "mrpRotationAlgorithm.h"
#include "msgPayloadDef/AttRefMsgF32Payload.h"
#include "msgPayloadDef/AttStateMsgF32Payload.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <stdint.h>
#include <Eigen/Core>

/*! @brief MRP Rotation adapter. */
class MrpRotation : public SysModel {
   public:
    MrpRotation() = default;
    ~MrpRotation() final = default;

    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    void setSigmaRR0(const Eigen::Vector3f& sigma);
    Eigen::Vector3f getSigmaRR0() const;
    void setOmegaRR0(const Eigen::Vector3f& omega);
    Eigen::Vector3f getOmegaRR0() const;

    Message<AttRefMsgF32Payload> attRefOutMsg;           //!< output message containing the Reference
    ReadFunctor<AttRefMsgF32Payload> attRefInMsg;        //!< guidance reference input message
    ReadFunctor<AttStateMsgF32Payload> desiredAttInMsg;  //!< incoming message containing the desired attitude set

   private:
    void rebuildAlgorithmConfig();

    Eigen::Vector3f sigma_RR0 = Eigen::Vector3f::Zero();
    Eigen::Vector3f omega_RR0_R = Eigen::Vector3f::Zero();
    MrpRotationAlgorithm algorithm{MrpRotationConfig::create(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), false)};
};

#endif
