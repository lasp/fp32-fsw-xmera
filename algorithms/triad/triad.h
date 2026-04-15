// SPDX-License-Identifier: ISC
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef F32XMERA_TRIAD_H
#define F32XMERA_TRIAD_H
#include <stdint.h>

#include <Eigen/Core>

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/AttRefMsgPayload.h>
#include <architecture/msgPayloadDef/BodyHeadingMsgPayload.h>
#include <architecture/msgPayloadDef/EphemerisMsgPayload.h>
#include <architecture/msgPayloadDef/InertialHeadingMsgPayload.h>
#include <architecture/msgPayloadDef/NavAttMsgPayload.h>
#include <architecture/msgPayloadDef/NavTransMsgPayload.h>


enum class CelestialBody { NotSun = 0, Sun = 1 };

enum class BodyAxisInput { inputBodyHeadingParameter = 0, inputBodyHeadingMsg = 1 };

enum class InertialAxisInput { inputInertialHeadingParameter = 0, inputInertialHeadingMsg = 1, inputEphemerisMsg = 2 };

class Triad : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    ReadFunctor<NavAttMsgPayload> attNavInMsg;
    ReadFunctor<BodyHeadingMsgPayload> bodyHeadingInMsg;
    ReadFunctor<InertialHeadingMsgPayload> inertialHeadingInMsg;
    ReadFunctor<NavTransMsgPayload> transNavInMsg;
    ReadFunctor<EphemerisMsgPayload> ephemerisInMsg;
    Message<AttRefMsgPayload> attRefOutMsg;

    void setA1Hat_B(const Eigen::Vector3d& a1Hat_B);
    const Eigen::Vector3d getA1Hat_B() const;

    void setH1Hat_B(const Eigen::Vector3d& h1Hat_B);
    const Eigen::Vector3d getH1Hat_B() const;

    void setHHat_N(const Eigen::Vector3d& hHat_N);
    const Eigen::Vector3d getHHat_N() const;

    void setCelestialBodyInput(const CelestialBody& celestialBodyInput);
    const CelestialBody getCelestialBodyInput() const;

    void setBodyAxisInput(const BodyAxisInput& bodyAxisInput);
    const BodyAxisInput getBodyAxisInput() const;

    void setInertialAxisInput(const InertialAxisInput& inertialAxisInput);
    const InertialAxisInput getInertialAxisInput() const;

   private:
    Eigen::Vector3d a1Hat_B = Eigen::Vector3d::Zero();
    Eigen::Vector3d h1Hat_B = Eigen::Vector3d::Zero();
    Eigen::Vector3d hHat_N = Eigen::Vector3d::Zero();
    CelestialBody celestialBodyInput{};
    BodyAxisInput bodyAxisInput{};
    InertialAxisInput inertialAxisInput{};
};

#endif
