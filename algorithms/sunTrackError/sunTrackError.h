#ifndef F32XMERA_SUN_TRACK_ERROR_H
#define F32XMERA_SUN_TRACK_ERROR_H

#include "msgPayloadDef/AttRefMsgF32Payload.h"
#include "msgPayloadDef/EphemerisMsgF32Payload.h"
#include "msgPayloadDef/NavAttMsgF32Payload.h"
#include "msgPayloadDef/NavTransMsgF32Payload.h"
#include "sunTrackErrorAlgorithm.h"
#include <architecture/messaging/messaging.h>
#include <stdint.h>
#include <Eigen/Core>
#include <memory>

/*!@brief Module to compute the attitude tracking error for sun avoidance.
 */
class SunTrackError final : public SysModel {
   public:
    SunTrackError() = default;
    ~SunTrackError() override = default;

    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    void reconfigure() const;
    void reInitialize();

    // Phase 1: public config properties -- set before reset().
    Eigen::Vector3f sensitiveHat_B = Eigen::Vector3f::Zero();  //!< [-] body vector to exclude from the Sun
    float angleRate = 0.0F;                                    //!< [r/s] rate at which we maneuver to Sun point

    ReadFunctor<NavAttMsgF32Payload> attNavInMsg;        //!< input msg measured attitude
    ReadFunctor<AttRefMsgF32Payload> attRefInMsg;        //!< input msg of reference attitude
    ReadFunctor<NavTransMsgF32Payload> transNavInMsg;    //!< input msg measured position
    ReadFunctor<EphemerisMsgF32Payload> ephemerisInMsg;  //!< input ephemeris msg
    Message<AttRefMsgF32Payload> attRefOutMsg;           //!< output msg of the maneuver-adjusted reference

   private:
    SunTrackErrorConfig toConfig() const;
    std::unique_ptr<SunTrackErrorAlgorithm> algorithm = nullptr;
};

#endif
