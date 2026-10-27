#ifndef F32XMERA_CONVERT_ST_PLATFORM_TO_BODY_H
#define F32XMERA_CONVERT_ST_PLATFORM_TO_BODY_H

#include "convertStPlatformToBodyAlgorithm.h"
#include "msgPayloadDef/STAttMsgF32Payload.h"
#include "msgPayloadDef/STSensorMsgF32Payload.h"

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>

#include <Eigen/Core>

#include <stdint.h>

#include <memory>

/*! @brief Convert STSensorMsgF32Payload to STAttMsgF32Payload Class */
class ConvertStPlatformToBody : public SysModel {
   public:
    ConvertStPlatformToBody() = default;
    ~ConvertStPlatformToBody() override = default;

    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    void reconfigure() const;

    Eigen::Vector3f sigma_CB = Eigen::Vector3f::Zero();  //!< [-] MRP of the case frame relative to the body frame

    ReadFunctor<STSensorMsgF32Payload> stSensorInMsg;  //!< Input msg
    Message<STAttMsgF32Payload> stAttOutMsg;           //!< Output msg

   private:
    ConvertStPlatformToBodyConfig toConfig() const;
    std::unique_ptr<ConvertStPlatformToBodyAlgorithm> algorithm = nullptr;
};

#endif
