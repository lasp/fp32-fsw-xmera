/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
*/

#ifndef F32XIMERA_RATE_CONTROL_ALGORITHM_H
#define F32XIMERA_RATE_CONTROL_ALGORITHM_H

#include "msgPayloadDef/AttGuidMsgF32Payload.h"
#include "msgPayloadDef/CmdTorqueBodyMsgF32Payload.h"
#include "msgPayloadDef/VehicleConfigMsgF32Payload.h"

#include <Eigen/Core>

class RateControlAlgorithm {
   public:
    CmdTorqueBodyMsgF32Payload update(AttGuidMsgF32Payload attGuidIn) const;
    void setSpacecraftInertia(VehicleConfigMsgF32Payload vehicleConfigIn);
    void setDerivativeGainP(float DerivativeGainP);
    float getDerivativeGainP() const;
    void setKnownTorquePntB_B(const Eigen::Vector3f& TorquePntB_B);
    const Eigen::Vector3f& getKnownTorquePntB_B() const;

   private:
    float P{};  //!< [N*m*s] Rate error feedback gain applied
    Eigen::Vector3f knownTorquePntB_B{
        Eigen::Vector3f::Zero()};  //!< [N*m] Known external torque expressed in body frame components
    Eigen::Matrix3f ISCPntB_B{
        Eigen::Matrix3f::Identity()};  //!< [kg*m^2] Spacecraft inertia about point B expressed in body frame components
};

#endif
