#include "celestialTwoBodyPoint.h"
#include "architecture/utilities/eigenSupport.h"
#include "utilities/xmera/xmeraLifecycleException.h"

#include <stdexcept>

void CelestialTwoBodyPoint::reset(const uint64_t callTime) {
    this->secCelBodyIsLinked = this->secCelBodyInMsg.isLinked();

    // check if required input messages have been included
    if (!this->transNavInMsg.isLinked()) {
        throw std::invalid_argument("celestialTwoBodyPoint.transNavInMsg was not linked.");
    }
    if (!this->celBodyInMsg.isLinked()) {
        throw std::invalid_argument("celestialTwoBodyPoint.celBodyInMsg was not linked.");
    }

    // Phase 2: Validate config and create algorithm
    auto config =
        CelestialTwoBodyPointConfig::create(this->singularityThreshold, this->rateThreshold, this->secCelBodyIsLinked);
    this->algorithm = std::make_unique<CelestialTwoBodyPointAlgorithm>(config);
}

/*! This method reads the input messages, computes the two-body celestial pointing attitude
 reference, and writes the attitude reference output message.
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void CelestialTwoBodyPoint::updateState(const uint64_t callTime) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("CelestialTwoBodyPoint reset() has not been called.");
    }

    const NavTransMsgF32Payload transNavIn = this->transNavInMsg();
    const EphemerisMsgF32Payload celBodyIn = this->celBodyInMsg();
    const Eigen::Vector3d r_BN_N = cArrayToEigenVector3<double>(transNavIn.r_BN_N);
    const Eigen::Vector3d v_BN_N = cArrayToEigenVector3<double>(transNavIn.v_BN_N);
    const Eigen::Vector3d r_celBody_N = cArrayToEigenVector3<double>(celBodyIn.r_BdyZero_N);
    const Eigen::Vector3d v_celBody_N = cArrayToEigenVector3<double>(celBodyIn.v_BdyZero_N);

    Eigen::Vector3d r_secCelBody_N = Eigen::Vector3d::Zero();
    Eigen::Vector3d v_secCelBody_N = Eigen::Vector3d::Zero();
    if (this->secCelBodyIsLinked) {
        const EphemerisMsgF32Payload secCelBodyIn = this->secCelBodyInMsg();
        r_secCelBody_N = cArrayToEigenVector3<double>(secCelBodyIn.r_BdyZero_N);
        v_secCelBody_N = cArrayToEigenVector3<double>(secCelBodyIn.v_BdyZero_N);
    }

    const CelestialTwoBodyPointOutput out =
        this->algorithm->update(r_celBody_N, v_celBody_N, r_secCelBody_N, v_secCelBody_N, r_BN_N, v_BN_N);

    /*! - Write the output message */
    AttRefMsgF32Payload attRefOut{};
    eigenVectorToCArray(out.sigma_RN, attRefOut.sigma_RN);
    eigenVectorToCArray(out.omega_RN_N, attRefOut.omega_RN_N);
    eigenVectorToCArray(out.domega_RN_N, attRefOut.domega_RN_N);
    this->attRefOutMsg.write(attRefOut, this->moduleID, callTime);
}
