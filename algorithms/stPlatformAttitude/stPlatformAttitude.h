#ifndef F32XMERA_ST_PLATFORM_ATTITUDE_H
#define F32XMERA_ST_PLATFORM_ATTITUDE_H

#include "msgPayloadDef/STAttMsgF32Payload.h"
#include "msgPayloadDef/StPlatformAttitudeMsgF32Payload.h"
#include "stPlatformAttitudeAlgorithm.h"

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>

#include <Eigen/Core>

#include <stdint.h>

#include <memory>

/*! @brief Convert StPlatformAttitudeMsgF32Payload to STAttMsgF32Payload Class */
class StPlatformAttitude : public SysModel {
   public:
    StPlatformAttitude() = default;
    ~StPlatformAttitude() override = default;

    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    void reconfigure() const;

    Eigen::Vector3f sigma_PB = Eigen::Vector3f::Zero();  //!< [-] MRP of the platform frame relative to the body frame

    ReadFunctor<StPlatformAttitudeMsgF32Payload> stSensorInMsg;  //!< Input msg
    Message<STAttMsgF32Payload> stAttOutMsg;                     //!< Output msg

   private:
    StPlatformAttitudeConfig toConfig() const;
    std::unique_ptr<StPlatformAttitudeAlgorithm> algorithm = nullptr;
};

#endif
