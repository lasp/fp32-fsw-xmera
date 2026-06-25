#ifndef F32XMERA_SUN_SAFE_POINT_H
#define F32XMERA_SUN_SAFE_POINT_H

#include "msgPayloadDef/AttGuidMsgF32Payload.h"
#include "msgPayloadDef/FilterResidualsMsgF32Payload.h"
#include "msgPayloadDef/NavAttMsgF32Payload.h"
#include "sunSafePointAlgorithm.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <stdint.h>
#include <Eigen/Core>
#include <array>
#include <memory>

/*! @brief Sun safe point attitude guidance class. */
class SunSafePoint final : public SysModel {
   public:
    SunSafePoint() = default;
    ~SunSafePoint() = default;

    void reset(uint64_t currentSimNanos) override;
    void updateState(uint64_t currentSimNanos) override;
    void reConfigure();
    void reInitialize();

    void setRotation(uint32_t index, const RotationProperties& rotation);
    RotationProperties getRotation(uint32_t index) const;

    Eigen::Vector3f sHatBdyCmd{0.0F, 0.0F, 1.0F};         //!< [-] commanded body vector to point at the sun
    float sunAxisSpinRate{};                              //!< [rad/s] constant spin rate about the sun heading vector
    Eigen::Vector3f omega_RN_B{Eigen::Vector3f::Zero()};  //!< [rad/s] fallback rate when no sun is available
    int observationThreshold{4};                          //!< [-] CSS count at or above which to transition to pointing

    ReadFunctor<NavAttMsgF32Payload> rateInMsg;                      //!< Body angular velocity input message
    ReadFunctor<NavAttMsgF32Payload> sunDirectionInMsg;              //!< Sun direction input message
    ReadFunctor<FilterResidualsMsgF32Payload> filterResidualsInMsg;  //!< Filter residuals input (CSS observation count)
    Message<AttGuidMsgF32Payload> attGuidanceOutMsg;                 //!< Attitude guidance output message

   private:
    std::unique_ptr<SunSafePointAlgorithm> algorithm = nullptr;
    std::array<RotationProperties, kNumRotations> rotations{};
};

#endif
