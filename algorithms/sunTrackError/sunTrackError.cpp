#include "sunTrackError.h"
#include "utilities/fsw/eigenSupport.h"
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
    const NavAttMsgF32Payload nav = this->attNavInMsg();  //!< attitude navigation message
    const AttRefMsgF32Payload ref = this->attRefInMsg();  //!< reference guidance message

    const SunTrackErrorNavAttInputs navInputs{cArrayToEigenVector3(nav.sigma_BN), cArrayToEigenVector3(nav.omega_BN_B)};
    const SunTrackErrorAttRefInputs refInputs{cArrayToEigenVector3(ref.sigma_RN),
                                              cArrayToEigenVector3(ref.omega_RN_N),
                                              cArrayToEigenVector3(ref.domega_RN_N)};

    Eigen::Vector3f r_BN_N = Eigen::Vector3f::Zero();  //!< spacecraft position
    Eigen::Vector3f r_SN_N = Eigen::Vector3f::Zero();  //!< sun position
    if (this->transNavInMsg.isLinked() && this->ephemerisInMsg.isLinked()) {
        r_BN_N = cArrayToEigenVector3(this->transNavInMsg().r_BN_N).cast<float>();
        r_SN_N = cArrayToEigenVector3(this->ephemerisInMsg().r_BdyZero_N).cast<float>();
    }

    const SunTrackErrorOutput out = this->algorithm.update(navInputs, refInputs, r_BN_N, r_SN_N, callTime);

    AttGuidMsgF32Payload attGuid{};
    eigenVectorToCArray(out.sigma_BR, attGuid.sigma_BR);
    eigenVectorToCArray(out.omega_BR_B, attGuid.omega_BR_B);
    eigenVectorToCArray(out.omega_RN_B, attGuid.omega_RN_B);
    eigenVectorToCArray(out.domega_RN_B, attGuid.domega_RN_B);

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
