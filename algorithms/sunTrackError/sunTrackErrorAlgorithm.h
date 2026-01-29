#ifndef F32XMERA_SUN_TRACK_ERROR_ALGORITHM_H
#define F32XMERA_SUN_TRACK_ERROR_ALGORITHM_H

#include "msgPayloadDef/AttGuidMsgF32Payload.h"
#include "msgPayloadDef/AttRefMsgF32Payload.h"
#include "msgPayloadDef/EphemerisMsgF32Payload.h"
#include "msgPayloadDef/NavAttMsgF32Payload.h"
#include "msgPayloadDef/NavTransMsgF32Payload.h"
#include <Eigen/Core>
#include <stdint.h>

/*!@brief Module to compute the attitude tracking error for sun avoidance.
 */
class SunTrackErrorAlgorithm final {
   public:
    void reset();
    AttGuidMsgF32Payload update(AttRefMsgF32Payload& ref,
                                NavAttMsgF32Payload& nav,
                                NavTransMsgF32Payload& navTrans,
                                EphemerisMsgF32Payload& celState,
                                bool navTransIsLinked,
                                bool ephemerisIsLinked,
                                uint64_t callTime);

    AttGuidMsgF32Payload computeSunTrackError(NavAttMsgF32Payload& nav,
                                              AttRefMsgF32Payload& ref,
                                              uint64_t callTime) const;
    void setSigma_R0R(const Eigen::Vector3f& sigma);
    Eigen::Vector3f getSigma_R0R() const;
    void setSensitiveHat_B(const Eigen::Vector3f& sensitiveDirection);
    Eigen::Vector3f getSensitiveHat_B() const;
    void setAngleRate(float rate);
    float getAngleRate() const;

   private:
    Eigen::Vector3f sigma_R0R{Eigen::Vector3f::Zero()};  /*!< MRP from corrected reference frame to original frame R0
                                                            This is the same as [BcB] going from primary body frame B
                                                            to the corrected body frame Bc */
    Eigen::Vector3f sensitiveHat_B{Eigen::Vector3f::Zero()};  //!< [-] Vehicle body vector to exclude from sun*/
    float angleRate{};                //!< [r/s] The rate at which we maneuver to Sun point*/
    Eigen::Vector3f mnvrAxis_B{Eigen::Vector3f::Zero()};  //!< [-] Eigen axis that we are maneuvering on*/
    float angleStart{};           //!< [r] The angle remaining in the attitude maneuver*/
    bool maneuverInitialized{};     //!< [-] Flag indicating if maneuver has been set*/
    uint64_t mnvrStartTime{};      //!< [ns] Time at which the maneuver was begun*/
};

#endif
