#ifndef F32XMERA_SUN_TRACK_ERROR_H
#define F32XMERA_SUN_TRACK_ERROR_H

#include "msgPayloadDef/AttGuidMsgF32Payload.h"
#include "msgPayloadDef/AttRefMsgF32Payload.h"
#include "msgPayloadDef/EphemerisMsgF32Payload.h"
#include "msgPayloadDef/NavAttMsgF32Payload.h"
#include "msgPayloadDef/NavTransMsgF32Payload.h"
#include "sunTrackErrorAlgorithm.h"
#include <architecture/messaging/messaging.h>
#include <stdint.h>
#include <Eigen/Core>

/*!@brief Module to compute the attitude tracking error for sun avoidance.
 */
class SunTrackError final : public SysModel {
   public:
    SunTrackError() = default;
    ~SunTrackError() override = default;

    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    void setSigma_R0R(const Eigen::Vector3f& sigma);
    Eigen::Vector3f getSigma_R0R() const;
    void setSensitiveHat_B(const Eigen::Vector3f& sensitiveDirection);
    Eigen::Vector3f getSensitiveHat_B() const;
    void setAngleRate(float rate);
    float getAngleRate() const;

    ReadFunctor<NavAttMsgF32Payload> attNavInMsg;        //!< input msg measured attitude
    ReadFunctor<AttRefMsgF32Payload> attRefInMsg;        //!< input msg of reference attitude
    ReadFunctor<NavTransMsgF32Payload> transNavInMsg;    //!< input msg measured position
    ReadFunctor<EphemerisMsgF32Payload> ephemerisInMsg;  //!< input ephemeris msg
    Message<AttGuidMsgF32Payload> attGuidOutMsg;         //!< output msg of attitude guidance

   private:
    SunTrackErrorAlgorithm algorithm{};
};

#endif
