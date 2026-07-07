#include "sunTrackError.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/xmera/xmeraLifecycleException.h"
#include <stdexcept>

/*! Validate that the required input messages are linked, build the algorithm's configuration from
 the adapter's stored properties (which captures whether the optional trans/ephemeris messages are
 connected), and (re)construct the embedded algorithm.
 @param callTime The clock time at which the function was called (nanoseconds).
 */
void SunTrackError::reset(uint64_t callTime) {
    // check if the required input messages are included
    if (!this->attRefInMsg.isLinked()) {
        throw std::invalid_argument("sunTrackError.attRefInMsg wasn't connected.");
    }
    if (!this->attNavInMsg.isLinked()) {
        throw std::invalid_argument("sunTrackError.attNavInMsg wasn't connected.");
    }

    this->algorithm = std::make_unique<SunTrackErrorAlgorithm>(this->toConfig());
}

/*! Build a validated SunTrackErrorConfig from the adapter's stored properties. The computeAngleStart
 flag is derived from whether the optional trans/ephemeris messages are connected.
 @return SunTrackErrorConfig validated configuration.
 */
SunTrackErrorConfig SunTrackError::toConfig() const {
    const bool computeAngleStart = this->transNavInMsg.isLinked() && this->ephemerisInMsg.isLinked();
    return SunTrackErrorConfig::create(this->sensitiveHat_B, this->angleRate, computeAngleStart);
}

/*! Push a fresh configuration into the algorithm without re-seeding its runtime maneuver state.
 @return void
 */
void SunTrackError::reconfigure() const {
    if (!this->algorithm) {
        throw XmeraLifecycleException("SunTrackError reset() has not been called.");
    }
    this->algorithm->setConfig(this->toConfig());
}

/*! Re-seed the algorithm's runtime maneuver state so the maneuver reinitializes on the next update.
 @return void
 */
void SunTrackError::reInitialize() {
    if (!this->algorithm) {
        throw XmeraLifecycleException("SunTrackError reset() has not been called.");
    }
    this->algorithm->reInitialize();
}

/*! This method computes the attitude tracking error for sun avoidance
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void SunTrackError::updateState(uint64_t callTime) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("SunTrackError reset() has not been called.");
    }

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

    const SunTrackErrorOutput out = this->algorithm->update(navInputs, refInputs, r_BN_N, r_SN_N, callTime);

    AttGuidMsgF32Payload attGuid{};
    eigenVectorToCArray(out.sigma_BR, attGuid.sigma_BR);
    eigenVectorToCArray(out.omega_BR_B, attGuid.omega_BR_B);
    eigenVectorToCArray(out.omega_RN_B, attGuid.omega_RN_B);
    eigenVectorToCArray(out.domega_RN_B, attGuid.domega_RN_B);

    /*! write output message */
    this->attGuidOutMsg.write(&attGuid, this->moduleID, callTime);
}
