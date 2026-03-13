#ifndef F32XMERA_SUN_SAFE_POINT_ALGORITHM_H
#define F32XMERA_SUN_SAFE_POINT_ALGORITHM_H

#include "msgPayloadDef/AttGuidMsgF32Payload.h"
#include "msgPayloadDef/NavAttMsgF32Payload.h"
#include <stdint.h>
#include <Eigen/Dense>

/*! @brief Sun safe point attitude guidance algorithm class. */
class SunSafePointAlgorithm {
   public:
    SunSafePointAlgorithm() = default;   //!< Constructor
    ~SunSafePointAlgorithm() = default;  //!< Destructor

    void reset(uint64_t currentSimNanos);  //!< Reset member function
    AttGuidMsgF32Payload update(uint64_t currentSimNanos,
                                NavAttMsgF32Payload imuInMsg,
                                NavAttMsgF32Payload sunDirectionInMsg);  //!< Update member function

    float getMinUnitMag() const;       //!< Getter method for the minimally accepted sun body vector norm
    float getSmallAngle() const;       //!< Getter method for the small alignment tolerance angle near 0 or 180 degrees
    float getSunAxisSpinRate() const;  //!< Getter method for the desired constant spin rate about sun heading vector
    Eigen::Vector3f getOmega_RN_B()
        const;  //!< Getter method for the desired body rate vector if no sun direction is available
    Eigen::Vector3f getSHatBdyCmd() const;  //!< Getter method for the desired body vector to point at the sun
    void setMinUnitMag(float minUnitMag);  //!< Setter method for the minimally accepted sun body vector norm
    void setSmallAngle(
        float smallAngle);  //!< Setter method for the small alignment tolerance angle near 0 or 180 degrees
    void setSunAxisSpinRate(
        const float sunAxisSpinRate);  //!< Setter method for the desired constant spin rate about sun heading vector
    void setOmega_RN_B(const Eigen::Vector3f& omega_RN_B);  //!< Setter method for the desired body rate vector if no
                                                            //!< sun direction is available
    void setSHatBdyCmd(Eigen::Vector3f& sHatBdyCmd);  //!< Setter method for the desired body vector to point at the sun

   private:
    void computeAttGuidanceStates(float sHatNorm);  //!< Method for computing the attitude guidance states sigma_BR and
                                                    //!< omega_RN_B if a valid sun direction vector is available
    void computeHubAngularRateError(
        NavAttMsgF32Payload imuInMsg);  //!< Method for computing the hub angular rate error omega_BR_B
    bool sunDirectionIsAvailable(
        const float sHatNorm) const;  //!< Method for determining if a valid sun direction vector is available

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
