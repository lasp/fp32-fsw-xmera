// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef F32XMERA_FLYBY_POINT_H
#define F32XMERA_FLYBY_POINT_H

#include "flybyPointAlgorithm.h"
#include "msgPayloadDef/AttRefMsgF32Payload.h"
#include "msgPayloadDef/EphemerisMsgF32Payload.h"
#include "msgPayloadDef/NavTransMsgF32Payload.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/FlybyDiagnosticMsgPayload.h>
#include <Eigen/Dense>
#include <memory>

/*! @brief A class to perform flyby pointing */
class FlybyPoint final : public SysModel {
   public:
    FlybyPoint() = default;
    ~FlybyPoint() override = default;

    void reset(uint64_t currentSimNanos) override;
    void updateState(uint64_t currentSimNanos) override;
    std::tuple<Eigen::Vector3d, Eigen::Vector3d> readRelativeState();

    // Phase 1: public config properties — set before reset()
    double timeBetweenFilterData = 0.0;     //!< [s] time between two subsequent filter reads (0 = every message)
    float toleranceForCollinearity = 0.0F;  //!< [-] tolerance for collinear r/v singular condition
    int signOfOrbitNormalFrameVector = 1;   //!< [-] sign (+1 or -1) of the orbit normal frame vector
    float maxRateThreshold = 0.0F;          //!< [deg/s] max predicted rate; 0 disables check
    float maxAccelerationThreshold = 0.0F;  //!< [deg/s^2] max predicted acceleration; 0 disables check
    float positionKnowledgeSigma = 0.0F;    //!< [m] position knowledge sigma; 0 disables check

    ReadFunctor<NavTransMsgF32Payload> filterInMsg;              //!< input msg relative position w.r.t. asteroid
    ReadFunctor<EphemerisMsgF32Payload> asteroidEphemerisInMsg;  //!< input asteroid ephemeris msg
    Message<AttRefMsgF32Payload> attRefOutMsg;                   //!< Attitude reference output message
    Message<FlybyDiagnosticMsgPayload> flybyDiagnosticOutMsg;    //!< Flyby diagnostic output message

   private:
    std::unique_ptr<FlybyPointAlgorithm> algorithm = nullptr;
};

#endif
