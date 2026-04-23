/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#ifndef F32XIMERA_CONVERT_ST_PLATFORM_TO_BODY_H
#define F32XIMERA_CONVERT_ST_PLATFORM_TO_BODY_H

#include "convertStPlatformToBodyAlgorithm.h"

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/STAttMsgPayload.h>
#include <architecture/msgPayloadDef/STSensorMsgPayload.h>
#include <architecture/utilities/bskLogging.h>

#include <Eigen/Core>

#include <stdint.h>

/*! @brief Convert STSensorMsgPayload to STAttMsgPayload Class */
class ConvertStPlatformToBody : public SysModel {
   public:
    ConvertStPlatformToBody() = default;
    ~ConvertStPlatformToBody() override = default;

    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    void setDcmCB(const Eigen::Matrix3d& dcm_CB);
    Eigen::Matrix3d getDcmCB() const;

    ReadFunctor<STSensorMsgPayload> stSensorInMsg;  //!< Input msg
    Message<STAttMsgPayload> stAttOutMsg;           //!< Output msg

    BSKLogger bskLogger{};  //!< BSK Logging

   private:
    ConvertStPlatformToBodyAlgorithm algorithm{};
};

#endif
