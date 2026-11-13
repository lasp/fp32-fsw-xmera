#ifndef F32XMERA_MRP_ROTATION_H
#define F32XMERA_MRP_ROTATION_H

#include "mrpRotationAlgorithm.h"
#include "msgPayloadDef/AttRefMsgF32Payload.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <stdint.h>
#include <Eigen/Core>
#include <memory>

/*! @brief MRP Rotation adapter. */
class MrpRotation final : public SysModel {
   public:
    MrpRotation() = default;
    ~MrpRotation() override = default;

    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    void reconfigure() const;
    void reInitialize();

    // Phase 1: public config properties -- set before reset().
    Eigen::Vector3f sigma_RcR0 = Eigen::Vector3f::Zero();   //!< [-] fixed body correction, Rc wrt input reference R0
    Eigen::Vector3f sigma_RpRc = Eigen::Vector3f::Zero();   //!< [-] commanded pointing offset, Rp wrt Rc in body axes
    Eigen::Vector3f spinAxis_Rp = Eigen::Vector3f::Zero();  //!< [-] unit spin axis of R wrt Rp, Rp axes (must be set)
    float spinRate = 0.0F;                                  //!< [rad/s] signed spin rate of R wrt Rp about spinAxis_Rp
    float controlPeriod = 0.0F;  //!< [s] integration step used by the algorithm (must be > 0)

    Message<AttRefMsgF32Payload> attRefOutMsg;     //!< output message containing the Reference
    ReadFunctor<AttRefMsgF32Payload> attRefInMsg;  //!< guidance reference input message

   private:
    MrpRotationConfig toConfig() const;
    std::unique_ptr<MrpRotationAlgorithm> algorithm = nullptr;
};

#endif
