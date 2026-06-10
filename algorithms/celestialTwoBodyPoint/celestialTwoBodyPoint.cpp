#include "celestialTwoBodyPoint.h"
#include "architecture/utilities/eigenSupport.h"
#include "utilities/xmera/xmeraLifecycleException.h"

#include <stdexcept>

void CelestialTwoBodyPoint::reset(const uint64_t callTime) {
    if (!this->transNavInMsg.isLinked()) {
        throw std::invalid_argument("celestialTwoBodyPoint.transNavInMsg was not linked.");
    }
    if (!this->primaryCelBodyInMsg.isLinked()) {
        throw std::invalid_argument("celestialTwoBodyPoint.primaryCelBodyInMsg was not linked.");
    }
    if (!this->secondaryCelBodyInMsg.isLinked()) {
        throw std::invalid_argument("celestialTwoBodyPoint.secondaryCelBodyInMsg was not linked.");
    }

    // Phase 2: Validate config and create algorithm
    auto config = CelestialTwoBodyPointConfig::create(this->singularityThreshold);
    this->algorithm = std::make_unique<CelestialTwoBodyPointAlgorithm>(config);
}

CelestialTwoBodyPointConfig CelestialTwoBodyPoint::toConfig() const {
    return CelestialTwoBodyPointConfig::create(this->singularityThreshold);
}

void CelestialTwoBodyPoint::reconfigure() const {
    if (!this->algorithm) {
        throw XmeraLifecycleException("CelestialTwoBodyPoint reset() has not been called.");
    }

    this->algorithm->setConfig(this->toConfig());
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
    const EphemerisMsgF32Payload primaryCelBodyIn = this->primaryCelBodyInMsg();
    const EphemerisMsgF32Payload secondaryCelBodyIn = this->secondaryCelBodyInMsg();
    const Eigen::Vector3d r_BN_N = cArrayToEigenVector3<double>(transNavIn.r_BN_N);
    const Eigen::Vector3d v_BN_N = cArrayToEigenVector3<double>(transNavIn.v_BN_N);
    const Eigen::Vector3d r_PN_N = cArrayToEigenVector3<double>(primaryCelBodyIn.r_BdyZero_N);
    const Eigen::Vector3d v_PN_N = cArrayToEigenVector3<double>(primaryCelBodyIn.v_BdyZero_N);
    const Eigen::Vector3d r_SN_N = cArrayToEigenVector3<double>(secondaryCelBodyIn.r_BdyZero_N);
    const Eigen::Vector3d v_SN_N = cArrayToEigenVector3<double>(secondaryCelBodyIn.v_BdyZero_N);

    const CelestialTwoBodyPointOutput out = this->algorithm->update(r_PN_N, v_PN_N, r_SN_N, v_SN_N, r_BN_N, v_BN_N);

    /*! - Write the output message */
    AttRefMsgF32Payload attRefOut{};
    eigenVectorToCArray(out.sigma_RN, attRefOut.sigma_RN);
    eigenVectorToCArray(out.omega_RN_N, attRefOut.omega_RN_N);
    eigenVectorToCArray(out.domega_RN_N, attRefOut.domega_RN_N);
    this->attRefOutMsg.write(attRefOut, this->moduleID, callTime);
}
