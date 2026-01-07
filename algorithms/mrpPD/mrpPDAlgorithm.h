#ifndef XMERAF32_MRP_PD_ALGORITHM_H
#define XMERAF32_MRP_PD_ALGORITHM_H

#include <stdint.h>
#include <stdexcept>

#include <Eigen/Dense>

#include "msgPayloadDef/AttGuidMsgF32Payload.h"
#include "msgPayloadDef/CmdTorqueBodyMsgF32Payload.h"
#include "msgPayloadDef/VehicleConfigMsgF32Payload.h"

/*! @brief MRP PD control algorithm class. */
class MrpPDAlgorithm {
   public:
    MrpPDAlgorithm() = default;
    ~MrpPDAlgorithm() = default;

    CmdTorqueBodyMsgF32Payload update(uint64_t currentSimNanos, AttGuidMsgF32Payload guidInMsg);
    void setSpacecraftInertia(VehicleConfigMsgF32Payload vehConfigInMsg);
    void setDerivativeGainP(float P);
    float getDerivativeGainP() const;
    void setKnownTorquePntB_B(Eigen::Vector3f& knownTorquePntB_B);
    const Eigen::Vector3f& getKnownTorquePntB_B() const;
    void setProportionalGainK(float K);
    float getProportionalGainK() const;

   private:
    float K{};                           //!< [rad/s] Proportional gain applied to MRP errors
    float P{};                           //!< [N*m*s] Rate error feedback gain applied
    Eigen::Vector3f knownTorquePntB_B{};  //!< [N*m] Known external torque expressed in body frame components
    Eigen::Matrix3f ISCPntB_B =
        Eigen::Matrix3f::Identity();  //!< [kg*m^2] Spacecraft inertia about point B expressed in body frame components
};

#endif
