#ifndef F32XMERA_SUN_SAFE_POINT_ALGORITHM_H
#define F32XMERA_SUN_SAFE_POINT_ALGORITHM_H

#include "sunSafePointTypes.h"
#include <Eigen/Core>

/*! @brief Structure containing the sun safe point attitude guidance output */
struct SunSafePointOutput {
    Eigen::Vector3f sigma_BR;    //!< attitude error (MRPs) of B relative to R
    Eigen::Vector3f omega_BR_B;  //!< [rad/s] body rate error of B relative to R in B frame
    Eigen::Vector3f omega_RN_B;  //!< [rad/s] reference frame rate of R relative to N in B frame
};

/*! @brief Sun safe point attitude guidance algorithm class. */
class SunSafePointAlgorithm {
   public:
    SunSafePointAlgorithm() = default;
    ~SunSafePointAlgorithm() = default;

    SunSafePointOutput update(const Eigen::Vector3f& vehSunPntBdy, const Eigen::Vector3f& omega_BN_B) const;

    float getSunAxisSpinRate() const;
    Eigen::Vector3f getOmega_RN_B() const;
    Eigen::Vector3f getSHatBdyCmd() const;
    void setSunAxisSpinRate(float rate);
    void setOmega_RN_B(const Eigen::Vector3f& omega);
    void setSHatBdyCmd(const Eigen::Vector3f& sHat);

   private:
    float sunAxisSpinRate{};       //!< [rad/s] Desired constant spin rate about sun heading vector
    Eigen::Vector3f omega_RN_B{};  //!< [rad/s] Desired body rate vector if no sun direction is available
    Eigen::Vector3f sHatBdyCmd{};  //!< Desired body vector to point at the sun
};

#endif
