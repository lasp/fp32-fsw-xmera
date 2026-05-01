#ifndef F32XMERA_SUN_SAFE_POINT_H
#define F32XMERA_SUN_SAFE_POINT_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include "msgPayloadDef/AttGuidMsgF32Payload.h"
#include "msgPayloadDef/NavAttMsgF32Payload.h"
#include "sunSafePointAlgorithm.h"
#include <stdint.h>
#include <Eigen/Core>

/*! @brief Sun safe point attitude guidance class. */
class SunSafePoint : public SysModel {
   public:
    SunSafePoint() = default;
    ~SunSafePoint() = default;

    void reset(uint64_t currentSimNanos) override;
    void updateState(uint64_t currentSimNanos) override;

    float getSunAxisSpinRate() const;
    Eigen::Vector3f getOmega_RN_B() const;
    Eigen::Vector3f getSHatBdyCmd() const;
    void setSunAxisSpinRate(float rate);
    void setOmega_RN_B(const Eigen::Vector3f& omega);
    void setSHatBdyCmd(const Eigen::Vector3f& sHat);

    ReadFunctor<NavAttMsgF32Payload> imuInMsg;           //!< IMU attitude guidance input message
    ReadFunctor<NavAttMsgF32Payload> sunDirectionInMsg;  //!< Sun attitude guidance input message
    Message<AttGuidMsgF32Payload> attGuidanceOutMsg;     //!< Attitude guidance output message

   private:
    SunSafePointAlgorithm algorithm{};
};

#endif
