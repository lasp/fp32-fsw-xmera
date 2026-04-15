// SPDX-License-Identifier: ISC
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef F32XMERA_TRIAD_H
#define F32XMERA_TRIAD_H
#include <stdint.h>

#include <Eigen/Core>

#include "triadAlgorithm.h"
#include "msgPayloadDef/AttRefMsgF32Payload.h"
#include "msgPayloadDef/BodyHeadingMsgF32Payload.h"
#include "msgPayloadDef/EphemerisMsgF32Payload.h"
#include "msgPayloadDef/InertialHeadingMsgF32Payload.h"
#include "msgPayloadDef/NavAttMsgF32Payload.h"
#include "msgPayloadDef/NavTransMsgF32Payload.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>

enum class BodyAxisInput : uint8_t { inputBodyHeadingParameter = 0, inputBodyHeadingMsg = 1 };

enum class InertialAxisInput : uint8_t {
    inputInertialHeadingParameter = 0,
    inputInertialHeadingMsg = 1,
    inputEphemerisMsg = 2
};

class Triad : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    ReadFunctor<NavAttMsgF32Payload> attNavInMsg;
    ReadFunctor<BodyHeadingMsgF32Payload> bodyHeadingInMsg;
    ReadFunctor<InertialHeadingMsgF32Payload> inertialHeadingInMsg;
    ReadFunctor<NavTransMsgF32Payload> transNavInMsg;
    ReadFunctor<EphemerisMsgF32Payload> ephemerisInMsg;
    Message<AttRefMsgF32Payload> attRefOutMsg;

    void setA1Hat_B(const Eigen::Vector3f& a1Hat_B);
    Eigen::Vector3f getA1Hat_B() const;

    void setH1Hat_B(const Eigen::Vector3f& h1Hat_B);
    Eigen::Vector3f getH1Hat_B() const;

    void setHHat_N(const Eigen::Vector3f& hHat_N);
    Eigen::Vector3f getHHat_N() const;

    void setCelestialBodyInput(const CelestialBody& celestialBodyInput);
    CelestialBody getCelestialBodyInput() const;

   private:
    TriadAlgorithm algorithm{};
    Eigen::Vector3f h1Hat_B = Eigen::Vector3f::Zero();
    Eigen::Vector3f hHat_N = Eigen::Vector3f::Zero();
    CelestialBody celestialBodyInput{};
    BodyAxisInput bodyAxisInput{};
    InertialAxisInput inertialAxisInput{};
};

#endif
