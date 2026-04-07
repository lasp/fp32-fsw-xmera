/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
*/

#ifndef F32XMERA_RATE_CONTROL_ALGORITHM_H
#define F32XMERA_RATE_CONTROL_ALGORITHM_H

#include <Eigen/Core>

class RateControlAlgorithm {
   public:
    Eigen::Vector3f update(const Eigen::Vector3f& omega_BR_B, const Eigen::Vector3f& domega_RN_B) const;
    void setSpacecraftInertia(const Eigen::Matrix3f& spacecraftInertia);
    void setDerivativeGainP(float derivativeGainP);  //!< [-] non-negative derivative gain
    float getDerivativeGainP() const;
    void setKnownTorquePntB_B(
        const Eigen::Vector3f& torquePntB_B);  //!< [N*m] Known external torque expressed in body frame components
    const Eigen::Vector3f& getKnownTorquePntB_B() const;

   private:
    float P{};  //!< [N*m*s] Rate error feedback gain applied
    Eigen::Vector3f knownTorquePntB_B{
        Eigen::Vector3f::Zero()};  //!< [N*m] Known external torque expressed in body frame components
    Eigen::Matrix3f ISCPntB_B{
        Eigen::Matrix3f::Identity()};  //!< [kg*m^2] Spacecraft inertia about point B expressed in body frame components
};

#endif
