// SPDX-License-Identifier: ISC
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef F32XMERA_HILL_POINT_H
#define F32XMERA_HILL_POINT_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/AttRefMsgPayload.h>
#include <architecture/msgPayloadDef/EphemerisMsgPayload.h>
#include <architecture/msgPayloadDef/NavTransMsgPayload.h>
#include <Eigen/Core>

/*! @brief Hill Point attitude guidance class. */
class HillPoint : public SysModel {
   public:
    HillPoint() = default;
    ~HillPoint() override = default;

    void reset(uint64_t currentSimNanos) override;
    void updateState(uint64_t currentSimNanos) override;

    ReadFunctor<NavTransMsgPayload> transNavInMsg;
    ReadFunctor<EphemerisMsgPayload> celBodyInMsg;
    Message<AttRefMsgPayload> attRefOutMsg;

   private:
    int planetMsgIsLinked{};

    static void computeHillPointingReference(Eigen::Vector3d r_BN_N,
                                             Eigen::Vector3d v_BN_N,
                                             Eigen::Vector3d celBdyPositionVector,
                                             Eigen::Vector3d celBdyVelocityVector,
                                             AttRefMsgPayload* attRefOut);
};

#endif
