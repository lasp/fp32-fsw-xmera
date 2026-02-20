/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
*/

#ifndef F32XIMERA_RATE_CONTROL_ALGORITHM_H
#define F32XIMERA_RATE_CONTROL_ALGORITHM_H

#include <Eigen/Core>

struct InputGuidanceData {
    Eigen::Vector3f omega_BR_B =
        Eigen::Vector3f::Zero();  //!< [r/s]  Current body error estimate of B relateive to R in B frame compoonents */
    Eigen::Vector3f omega_RN_B = Eigen::Vector3f::Zero();   //!< [r/s]  Reference frame rate vector of the of R relative
                                                            //!< to N in B frame components */
    Eigen::Vector3f domega_RN_B = Eigen::Vector3f::Zero();  //!< [r/s2] Reference frame inertial body acceleration of R
                                                            //!< relative to N in B frame components */
};

class RateControlAlgorithm {
   public:
    Eigen::Vector3f update(const InputGuidanceData& attGuidIn) const;
    void setSpacecraftInertia(const Eigen::Matrix3f& spacecraftInertia);
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
