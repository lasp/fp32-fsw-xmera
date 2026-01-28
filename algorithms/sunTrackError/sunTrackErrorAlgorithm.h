#ifndef F32XMERA_SUN_TRACK_ERROR_ALGORITHM_H
#define F32XMERA_SUN_TRACK_ERROR_ALGORITHM_H

#include <architecture/msgPayloadDef/AttGuidMsgPayload.h>
#include <architecture/msgPayloadDef/AttRefMsgPayload.h>
#include <architecture/msgPayloadDef/EphemerisMsgPayload.h>
#include <architecture/msgPayloadDef/NavAttMsgPayload.h>
#include <architecture/msgPayloadDef/NavTransMsgPayload.h>
#include <Eigen/Core>
#include <stdint.h>

/*!@brief Module to compute the attitude tracking error for sun avoidance.
 */
class SunTrackErrorAlgorithm final {
   public:
    void reset();
    AttGuidMsgPayload update(AttRefMsgPayload& ref,
                             NavAttMsgPayload& nav,
                             NavTransMsgPayload& navTrans,
                             EphemerisMsgPayload& celState,
                             bool navTransIsLinked,
                             bool ephemerisIsLinked,
                             uint64_t callTime);

    AttGuidMsgPayload computeSunTrackError(NavAttMsgPayload& nav,
                                           AttRefMsgPayload& ref,
                                           uint64_t callTime) const;
    void setSigma_R0R(const Eigen::Vector3d& sigma);
    Eigen::Vector3d getSigma_R0R() const;
    void setSensitiveHat_B(const Eigen::Vector3d& sensitiveDirection);
    Eigen::Vector3d getSensitiveHat_B() const;
    void setAngleRate(double rate);
    double getAngleRate() const;

   private:
    Eigen::Vector3d sigma_R0R{Eigen::Vector3d::Zero()};  /*!< MRP from corrected reference frame to original frame R0
                                                            This is the same as [BcB] going from primary body frame B
                                                            to the corrected body frame Bc */
    Eigen::Vector3d sensitiveHat_B{Eigen::Vector3d::Zero()};  //!< [-] Vehicle body vector to exclude from sun*/
    double angleRate{};                //!< [r/s] The rate at which we maneuver to Sun point*/
    Eigen::Vector3d mnvrAxis_B{Eigen::Vector3d::Zero()};  //!< [-] Eigen axis that we are maneuvering on*/
    double angleStart{};           //!< [r] The angle remaining in the attitude maneuver*/
    bool maneuverInitialized{};     //!< [-] Flag indicating if maneuver has been set*/
    uint64_t mnvrStartTime{};      //!< [ns] Time at which the maneuver was begun*/
};

#endif
