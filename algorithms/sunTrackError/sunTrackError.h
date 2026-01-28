#ifndef F32XMERA_SUN_TRACK_ERROR_H
#define F32XMERA_SUN_TRACK_ERROR_H

#include "sunTrackErrorAlgorithm.h"
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/AttGuidMsgPayload.h>
#include <architecture/msgPayloadDef/AttRefMsgPayload.h>
#include <architecture/msgPayloadDef/EphemerisMsgPayload.h>
#include <architecture/msgPayloadDef/NavAttMsgPayload.h>
#include <architecture/msgPayloadDef/NavTransMsgPayload.h>
#include <Eigen/Core>
#include <stdint.h>

/*!@brief Module to compute the attitude tracking error for sun avoidance.
 */
class SunTrackError final : public SysModel {
   public:
    SunTrackError() = default;
    ~SunTrackError() override = default;

    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    void setSigma_R0R(const Eigen::Vector3d& sigma);
    Eigen::Vector3d getSigma_R0R() const;
    void setSensitiveHat_B(const Eigen::Vector3d& sensitiveDirection);
    Eigen::Vector3d getSensitiveHat_B() const;
    void setAngleRate(double rate);
    double getAngleRate() const;

    ReadFunctor<NavAttMsgPayload> attNavInMsg;        //!< input msg measured attitude
    ReadFunctor<AttRefMsgPayload> attRefInMsg;        //!< input msg of reference attitude
    ReadFunctor<NavTransMsgPayload> transNavInMsg;    //!< input msg measured position
    ReadFunctor<EphemerisMsgPayload> ephemerisInMsg;  //!< input ephemeris msg
    Message<AttGuidMsgPayload> attGuidOutMsg;         //!< output msg of attitude guidance

   private:
    SunTrackErrorAlgorithm algorithm{};
};

#endif
