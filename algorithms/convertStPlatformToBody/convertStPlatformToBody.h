#ifndef F32XIMERA_CONVERT_ST_PLATFORM_TO_BODY_H
#define F32XIMERA_CONVERT_ST_PLATFORM_TO_BODY_H

#include "convertStPlatformToBodyAlgorithm.h"

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/STAttMsgPayload.h>
#include <architecture/msgPayloadDef/STSensorMsgPayload.h>

#include <Eigen/Core>

#include <stdint.h>

#include <memory>

/*! @brief Convert STSensorMsgPayload to STAttMsgPayload Class */
class ConvertStPlatformToBody : public SysModel {
   public:
    ConvertStPlatformToBody() = default;
    ~ConvertStPlatformToBody() override = default;

    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    Eigen::Matrix3f dcm_CB = Eigen::Matrix3f::Identity();  //!< [-] body-to-case mounting DCM (orthonormal, det +1)

    ReadFunctor<STSensorMsgPayload> stSensorInMsg;  //!< Input msg
    Message<STAttMsgPayload> stAttOutMsg;           //!< Output msg

   private:
    std::unique_ptr<ConvertStPlatformToBodyAlgorithm> algorithm = nullptr;
};

#endif
