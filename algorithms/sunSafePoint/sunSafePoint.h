#ifndef F32XMERA_SUN_SAFE_POINT_H
#define F32XMERA_SUN_SAFE_POINT_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include "msgPayloadDef/AttGuidMsgF32Payload.h"
#include "msgPayloadDef/NavAttMsgF32Payload.h"
#include "sunSafePointAlgorithm.h"
#include <stdint.h>
#include <Eigen/Dense>

/*! @brief Sun safe point attitude guidance class. */
class SunSafePoint : public SysModel {
   public:
    SunSafePoint() = default;   //!< Constructor
    ~SunSafePoint() = default;  //!< Destructor

    void reset(uint64_t currentSimNanos) override;        //!< Reset member function
    void updateState(uint64_t currentSimNanos) override;  //!< Update member function

    float getMinUnitMag() const;       //!< Getter method for the minimally accepted sun body vector norm
    float getSmallAngle() const;       //!< Getter method for the small alignment tolerance angle near 0 or 180 degrees
    float getSunAxisSpinRate() const;  //!< Getter method for the desired constant spin rate about sun heading vector
    Eigen::Vector3f getOmega_RN_B()
        const;  //!< Getter method for the desired body rate vector if no sun direction is available
    Eigen::Vector3f getSHatBdyCmd() const;        //!< Getter method for the desired body vector to point at the sun
    void setMinUnitMag(const float minUnitMag);  //!< Setter method for the minimally accepted sun body vector norm
    void setSmallAngle(
        const float smallAngle);  //!< Setter method for the small alignment tolerance angle near 0 or 180 degrees
    void setSunAxisSpinRate(
        const float sunAxisSpinRate);  //!< Setter method for the desired constant spin rate about sun heading vector
    void setOmega_RN_B(const Eigen::Vector3f& omega_RN_B);  //!< Setter method for the desired body rate vector if no
                                                            //!< sun direction is available
    void setSHatBdyCmd(Eigen::Vector3f& sHatBdyCmd);  //!< Setter method for the desired body vector to point at the sun

    ReadFunctor<NavAttMsgF32Payload> imuInMsg;           //!< IMU attitude guidance input message
    ReadFunctor<NavAttMsgF32Payload> sunDirectionInMsg;  //!< Sun attitude guidance input message
    Message<AttGuidMsgF32Payload> attGuidanceOutMsg;     //!< Attitude guidance output message

   private:
    SunSafePointAlgorithm algorithm{};  //!< Algorithm for sunSafePoint guidance logic (BSK-agnostic)
};

#endif
