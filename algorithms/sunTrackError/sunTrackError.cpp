#include "sunTrackError.h"
#include <stdexcept>

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 */
void SunTrackError::reset(uint64_t callTime) {
    // check if the required input messages are included
    if (!this->attRefInMsg.isLinked()) {
        throw std::invalid_argument("sunTrackError.attRefInMsg wasn't connected.");
    }
    if (!this->attNavInMsg.isLinked()) {
        throw std::invalid_argument("sunTrackError.attNavInMsg wasn't connected.");
    }

    const bool computeStartAngle = this->transNavInMsg.isLinked() && this->ephemerisInMsg.isLinked();

    this->algorithm.reset(computeStartAngle);
}

/*! This method computes the attitude tracking error for sun avoidance
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void SunTrackError::updateState(uint64_t callTime) {
    AttRefMsgF32Payload ref = this->attRefInMsg();  //!< reference guidance message
    NavAttMsgF32Payload nav = this->attNavInMsg();  //!< attitude navigation message
    NavTransMsgF32Payload navTrans{};    //!< spacecraft position
    EphemerisMsgF32Payload celState{};  //!< sun position

    if (this->transNavInMsg.isLinked() && this->ephemerisInMsg.isLinked()) {
        navTrans = this->transNavInMsg();
        celState = this->ephemerisInMsg();
    }

    AttGuidMsgF32Payload attGuid = this->algorithm.update(ref,
                                                          nav,
                                                          navTrans,
                                                          celState,
                                                          callTime);

    /*! write output message */
    this->attGuidOutMsg.write(&attGuid, this->moduleID, callTime);
}

/*! Set the MRP from corrected reference frame to original frame R0.
 @return void
 @param sigma [-] The MRP from corrected reference frame to original frame R0
*/
void SunTrackError::setSigma_R0R(const Eigen::Vector3f& sigma) { this->algorithm.setSigma_R0R(sigma); }

/*! Get the MRP from corrected reference frame to original frame R0.
 @return const Eigen::Vector3f
*/
Eigen::Vector3f SunTrackError::getSigma_R0R() const { return this->algorithm.getSigma_R0R(); }

/*! Set the direction to exclude from the Sun in body frame components.
 @return void
 @param sensitiveDirection [-] The direction to exclude from the Sun in body frame components
*/
void SunTrackError::setSensitiveHat_B(const Eigen::Vector3f& sensitiveDirection) {
    this->algorithm.setSensitiveHat_B(sensitiveDirection);
}

/*! Get the direction to exclude from the Sun in body frame components.
 @return const Eigen::Vector3f
*/
Eigen::Vector3f SunTrackError::getSensitiveHat_B() const { return this->algorithm.getSensitiveHat_B(); }

/*! Set the rate at which we maneuver to Sun point.
 @return void
 @param rate [rad/s] The rate at which we maneuver to Sun point
*/
void SunTrackError::setAngleRate(const float rate) { this->algorithm.setAngleRate(rate); }

/*! Get the rate at which we maneuver to Sun point.
 @return const float
*/
float SunTrackError::getAngleRate() const { return this->algorithm.getAngleRate(); }
