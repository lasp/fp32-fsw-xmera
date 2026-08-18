/*
    Thruster RW Momentum Management

 */

#include "thrMomentumManagementAlgorithm.h"

// [Nms] net RW momentum below this magnitude is treated as zero.
constexpr float kZeroMomentumTolerance = 1e-6F;

/*! Construct the algorithm from a validated configuration.
 @param config the validated configuration
 */
ThrMomentumManagementAlgorithm::ThrMomentumManagementAlgorithm(const ThrMomentumManagementConfig& config)
    : cfg(config) {
    this->setConfig(config);
}

/*! Install the validated configuration.
 @return void
 @param config the validated configuration
 */
void ThrMomentumManagementAlgorithm::setConfig(const ThrMomentumManagementConfig& config) { this->cfg = config; }

/*! The RW momentum level is assessed on every call to determine the torque that dumps the momentum held above
 the hsMin threshold.
 @return Eigen::Vector3f [Nm] the requested body-frame torque
 @param wheelSpeeds [r/s] current reaction wheel speeds
 */
Eigen::Vector3f ThrMomentumManagementAlgorithm::update(const Eigen::Vector<float, kMaxNumRw>& wheelSpeeds) const {
    /*! - compute net RW momentum magnitude */
    const ThrMomentumManagementRwArrayConfiguration& rwArrayConfig = this->cfg.getRwArrayConfiguration();
    Eigen::Vector3f hs_B = Eigen::Vector3f::Zero(); /* RW angular momentum */
    for (uint32_t i = 0; i < rwArrayConfig.numRW; i++) {
        hs_B += rwArrayConfig.JsList(i) * wheelSpeeds(i) * rwArrayConfig.GsMatrix_B.col(i);
    }
    const float hs = hs_B.norm(); /* net RW cluster angular momentum magnitude */

    /*! - the momentum held above the threshold, along the cluster momentum. It stays zero inside the deadband,
     which also avoids a 0/0 division when there is no momentum at all */
    const ThrMomentumManagementControlParameters& params = this->cfg.getControlParameters();
    Eigen::Vector3f hsExcess_B = Eigen::Vector3f::Zero(); /* [Nms] excess RW cluster momentum */
    if (hs >= params.hsMin && hs >= kZeroMomentumTolerance) {
        hsExcess_B = (hs - params.hsMin) * hs_B / hs;
    }

    /*! - the requested torque opposes the excess momentum, scaled by the feedback gain */
    return -params.K * hsExcess_B;
}
