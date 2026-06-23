// SPDX-License-Identifier: ISC
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef F32XMERA_TIME_CA_H
#define F32XMERA_TIME_CA_H

#include "timeClosestApproachAlgorithm.h"

#include "msgPayloadDef/FilterMsgF32Payload.h"
#include "msgPayloadDef/TimeClosestApproachMsgF32Payload.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <memory>

/*! @brief A class to perform time of closest approach estimation during a rectilinear flyby */
class TimeClosestApproach : public SysModel {
   public:
    TimeClosestApproach() = default;
    ~TimeClosestApproach() override = default;
    void reset(uint64_t callTime) override;
    void updateState(uint64_t currentSimNanos) override;

    ReadFunctor<FilterMsgF32Payload> filterInMsg;         //!< relative state and covariance input msg
    Message<TimeClosestApproachMsgF32Payload> tcaOutMsg;  //!< time of closest approach output message

   private:
    std::unique_ptr<TimeClosestApproachAlgorithm> algorithm = nullptr;
    void writeMessages(double tCA, double sigmaTca, uint64_t currentSimNanos);
};

#endif
