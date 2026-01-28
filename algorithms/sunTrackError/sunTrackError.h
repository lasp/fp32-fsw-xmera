
#ifndef F32XMERA_SUN_TRACK_ERROR_H
#define F32XMERA_SUN_TRACK_ERROR_H

#include <stdint.h>

#include <Eigen/Core>

#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/AttGuidMsgPayload.h>
#include <architecture/msgPayloadDef/AttRefMsgPayload.h>
#include <architecture/msgPayloadDef/EphemerisMsgPayload.h>
#include <architecture/msgPayloadDef/NavAttMsgPayload.h>
#include <architecture/msgPayloadDef/NavTransMsgPayload.h>

/*!@brief Module to compute the attitude tracking error for sun avoidance.
 */
class SunTrackError : public SysModel {
   public:
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

    void reset(uint64_t callTime) override;

    void updateState(uint64_t callTime) override;

   private:
    Eigen::Vector3d sigma_R0R;       /*!< MRP from corrected reference frame to original frame R0
                                    This is the same as [BcB] going from primary body frame B
                                    to the corrected body frame Bc */
    Eigen::Vector3d sensitiveHat_B;  //!< [-] Vehicle body vector to exclude from sun*/
    double angleRate;                //!< [r/s] The rate at which we maneuver to Sun point*/
    Eigen::Vector3d mnvrAxis_B;  //!< [-] Eigen axis that we are maneuvering on*/
    double angleStart;           //!< [r] The angle remaining in the attitude maneuver*/
    bool maneuverInitialized;     //!< [-] Flag indicating if maneuver has been set*/
    uint64_t mnvrStartTime;      //!< [ns] Time at which the maneuver was begun*/

    AttGuidMsgPayload computeSunTrackError(NavAttMsgPayload& nav,
                                           AttRefMsgPayload& ref,
                                           uint64_t callTime) const;
};

#endif
