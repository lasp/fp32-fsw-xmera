#include "hillPoint.h"
#include "utilities/fsw/eigenSupport.h"
#include <stdexcept>

namespace {
class XmeraLifecycleException : public std::runtime_error {
   public:
    using runtime_error::runtime_error;
};
}  // namespace

void HillPoint::reset(const uint64_t currentSimNanos) {
    if (!this->transNavInMsg.isLinked()) {
        throw std::invalid_argument("hillPoint.transNavInMsg wasn't connected.");
    }
    this->planetMsgIsLinked = this->celBodyInMsg.isLinked();
    auto config = HillPointConfig::create();
    this->algorithm = std::make_unique<HillPointAlgorithm>(config);
}

/*! Computes a Hill-frame attitude reference from the spacecraft's inertial position and velocity. */
void HillPoint::updateState(const uint64_t currentSimNanos) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("HillPoint reset() has not been called.");
    }

    const NavTransMsgF32Payload navData = this->transNavInMsg();
    const Eigen::Vector3d r_BN_N = cArrayToEigenVector3<double>(navData.r_BN_N);
    const Eigen::Vector3d v_BN_N = cArrayToEigenVector3<double>(navData.v_BN_N);

    Eigen::Vector3d r_planet_N = Eigen::Vector3d::Zero();
    Eigen::Vector3d v_planet_N = Eigen::Vector3d::Zero();
    if (this->planetMsgIsLinked) {
        const EphemerisMsgF32Payload primPlanet = this->celBodyInMsg();
        r_planet_N = cArrayToEigenVector3<double>(primPlanet.r_BdyZero_N);
        v_planet_N = cArrayToEigenVector3<double>(primPlanet.v_BdyZero_N);
    }

    const HillPointOutput out = this->algorithm->update(r_BN_N, v_BN_N, r_planet_N, v_planet_N);

    AttRefMsgF32Payload attRefOut = AttRefMsgF32Payload();
    eigenVectorToCArray(out.sigma_RN, attRefOut.sigma_RN);
    eigenVectorToCArray(out.omega_RN_N, attRefOut.omega_RN_N);
    eigenVectorToCArray(out.domega_RN_N, attRefOut.domega_RN_N);

    this->attRefOutMsg.write(&attRefOut, moduleID, currentSimNanos);
}
