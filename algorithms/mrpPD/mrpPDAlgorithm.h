#ifndef XMERAF32_MRP_PD_ALGORITHM_H
#define XMERAF32_MRP_PD_ALGORITHM_H

#include "../freestandingInvalidArgument.h"
#include "utilities/validInertiaCheck.h"
#include <stdint.h>
#include <Eigen/Core>

/*! @brief MRP PD control algorithm class. */
class MrpPDAlgorithm {
   public:
    MrpPDAlgorithm() = default;
    ~MrpPDAlgorithm() = default;

    Eigen::Vector3f update(const Eigen::Vector3f& sigma_BR,
                           const Eigen::Vector3f& omega_BR_B,
                           const Eigen::Vector3f& domega_RN_B) const;
    void setSpacecraftInertia(const Eigen::Matrix3f& inertia);
    Eigen::Matrix3f getSpacecraftInertia() const;
    void setDerivativeGainP(float P);
    float getDerivativeGainP() const;
    void setKnownTorquePntB_B(const Eigen::Vector3f& knownTorquePntB_B);
    const Eigen::Vector3f& getKnownTorquePntB_B() const;
    void setProportionalGainK(float K);
    float getProportionalGainK() const;

   private:
    float proportionalGain{};  //!< [rad/s] Proportional gain applied to MRP errors
    float feedbackGain{};      //!< [N*m*s] Rate error feedback gain applied
    Eigen::Vector3f knownTorquePntB_B =
        Eigen::Vector3f::Zero();  //!< [N*m] Known external torque expressed in body frame components
    Eigen::Matrix3f ISCPntB_B =
        Eigen::Matrix3f::Identity();  //!< [kg*m^2] Spacecraft inertia about point B expressed in body frame components
};

#endif
