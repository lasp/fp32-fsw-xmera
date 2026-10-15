#ifndef F32XMERA_DV_GUIDANCE_H
#define F32XMERA_DV_GUIDANCE_H

#include "dvGuidanceAlgorithm.h"
#include "msgPayloadDef/AttRefMsgF32Payload.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>

#include <stdint.h>
#include <Eigen/Core>
#include <memory>

/*! @brief Adapter for the delta-V guidance algorithm. */
class DvGuidance final : public SysModel {
   public:
    DvGuidance() = default;
    ~DvGuidance() override = default;

    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    void reconfigure() const;

    Eigen::Vector3f cmdDv_N = Eigen::Vector3f::Zero();     //!< [m/s] Commanded delta-V vector, inertial frame
    Eigen::Vector3f rHatPref_N = Eigen::Vector3f::Zero();  //!< [-] Preferred 2nd-axis unit direction, inertial frame

    Message<AttRefMsgF32Payload> attRefOutMsg;

   private:
    DvGuidanceConfig toConfig() const;
    std::unique_ptr<DvGuidanceAlgorithm> algorithm = nullptr;
};

#endif
