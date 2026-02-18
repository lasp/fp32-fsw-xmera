/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#ifndef F32XIMERA_INERTIAL_UKF_H
#define F32XIMERA_INERTIAL_UKF_H

#include "inertialUKFAlgorithm.h"
#include "msgPayloadDef/AccDataMsgF32Payload.h"
#include "msgPayloadDef/InertialFilterMsgF32Payload.h"
#include "msgPayloadDef/NavAttMsgF32Payload.h"
#include "msgPayloadDef/RWArrayConfigMsgF32Payload.h"
#include "msgPayloadDef/RWSpeedMsgF32Payload.h"
#include "msgPayloadDef/STAttMsgF32Payload.h"
#include "msgPayloadDef/VehicleConfigMsgF32Payload.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>

class InertialUKF : public SysModel {
   public:
    InertialUKF() = default;
    ~InertialUKF() = default;

    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    ReadFunctor<STAttMsgF32Payload> stAttInMsg;              //!< star tracker attitude input message
    ReadFunctor<AccDataMsgF32Payload> gyrBuffInMsg;          //!< gyro buffer input message
    ReadFunctor<RWSpeedMsgF32Payload> rwSpeedsInMsg;         //!< RW speeds input message
    ReadFunctor<RWArrayConfigMsgF32Payload> rwParamsInMsg;   //!< RW array configuration input message
    ReadFunctor<VehicleConfigMsgF32Payload> massPropsInMsg;  //!< vehicle mass properties input message
    Message<NavAttMsgF32Payload> navStateOutMsg;             //!< navigation attitude output message
    Message<InertialFilterMsgF32Payload> filtDataOutMsg;     //!< filter data output message
};

#endif
