#ifndef F32XMERA_SUN_SAFE_POINT_ALGORITHM_H
#define F32XMERA_SUN_SAFE_POINT_ALGORITHM_H

#include "sunSafePointTypes.h"
#include <Eigen/Core>

/*! @brief Sun safe point attitude guidance algorithm class. */
class SunSafePointAlgorithm {
   public:
    SunSafePointAlgorithm() = default;
    ~SunSafePointAlgorithm() = default;

    SunSafePointOutput update(const Eigen::Vector3f& vehSunPntBdy, const Eigen::Vector3f& omega_BN_B) const;

    float getSmallAngle() const;
    float getSunAxisSpinRate() const;
    Eigen::Vector3f getOmega_RN_B() const;
    Eigen::Vector3f getSHatBdyCmd() const;
    void setSmallAngle(float angle);
    void setSunAxisSpinRate(float rate);
    void setOmega_RN_B(const Eigen::Vector3f& omega);
    void setSHatBdyCmd(const Eigen::Vector3f& sHat);

   private:
    float smallAngle{};           //!< [rad] An angle value that specifies what is near 0 or 180 degrees (Must be >= 0)
    float sunAxisSpinRate{};      //!< [rad/s] Desired constant spin rate about sun heading vector
    Eigen::Vector3f omega_RN_B{};  //!< [rad/s] Desired body rate vector if no sun direction is available
    Eigen::Vector3f sHatBdyCmd{};  //!< Desired body vector to point at the sun
};

#endif
