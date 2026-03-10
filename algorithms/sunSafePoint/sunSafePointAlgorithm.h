#ifndef F32XMERA_SUN_SAFE_POINT_ALGORITHM_H
#define F32XMERA_SUN_SAFE_POINT_ALGORITHM_H

#include "msgPayloadDef/AttGuidMsgF32Payload.h"
#include "msgPayloadDef/NavAttMsgF32Payload.h"
#include <stdint.h>
#include <Eigen/Dense>

/*! @brief Sun safe point attitude guidance algorithm class. */
class SunSafePointAlgorithm {
   public:
    SunSafePointAlgorithm() = default;
    ~SunSafePointAlgorithm() = default;

    void reset(uint64_t currentSimNanos);
    AttGuidMsgF32Payload update(uint64_t currentSimNanos,
                                NavAttMsgF32Payload imuInMsg,
                                NavAttMsgF32Payload sunDirectionInMsg);

    float getMinUnitMag() const;
    float getSmallAngle() const;
    float getSunAxisSpinRate() const;
    Eigen::Vector3f getOmega_RN_B() const;
    Eigen::Vector3f getSHatBdyCmd() const;
    void setMinUnitMag(float minUnitMag);
    void setSmallAngle(float smallAngle);
    void setSunAxisSpinRate(const float sunAxisSpinRate);
    void setOmega_RN_B(const Eigen::Vector3f& omega_RN_B);
    void setSHatBdyCmd(Eigen::Vector3f& sHatBdyCmd);

   private:
    void computeAttGuidanceStates(float sHatNorm);
    void computeHubAngularRateError(NavAttMsgF32Payload imuInMsg);
    bool sunDirectionIsAvailable(const float sHatNorm) const;

    float minUnitMag{0.1f};        //!< The minimally acceptable norm of sun body vector (Must be positive)
    float smallAngle{};           //!< [rad] An angle value that specifies what is near 0 or 180 degrees (Must be >= 0)
    float sunAxisSpinRate{};      //!< [rad/s] Desired constant spin rate about sun heading vector
    Eigen::Vector3f omega_RN_B{};  //!< [rad/s] Desired body rate vector if no sun direction is available
    Eigen::Vector3f sHatBdyCmd{0.0f, 0.0f, 1.0f};  //!< Desired body vector to point at the sun
    Eigen::Vector3f eHat180_B{1.0f, 0.0f, 0.0f};   //!< Eigen axis to use if commanded axis is 180 from sun axis

    AttGuidMsgF32Payload attGuidanceOutBuffer;  //!< Attitude guidance output message buffer
    NavAttMsgF32Payload sunDirectionInBuffer;   //!< Sun attitude guidance input message buffer
};

#endif
