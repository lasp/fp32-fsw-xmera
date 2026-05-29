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

    // Phase 1: public config properties -- set before reset().
    Eigen::Vector3f sigma_RR0 = Eigen::Vector3f::Zero();    //!< [-] initial MRP attitude relative to input reference
    Eigen::Vector3f omega_RR0_R = Eigen::Vector3f::Zero();  //!< [rad/s] constant angular velocity in R-frame components
    float controlPeriod = 0.0F;  //!< [s] integration step used by the algorithm (must be > 0)

    Message<AttRefMsgF32Payload> attRefOutMsg;     //!< output message containing the Reference
    ReadFunctor<AttRefMsgF32Payload> attRefInMsg;  //!< guidance reference input message

   private:
    std::unique_ptr<MrpRotationAlgorithm> algorithm = nullptr;
};

#endif
