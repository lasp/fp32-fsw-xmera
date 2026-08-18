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

/*! The RW momentum level is assessed on every call to determine the angular momentum change that brings the
 cluster momentum down to hsMin.
 @return Eigen::Vector3f [Nms] the requested momentum change
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

    /*! - dumping is only required above the threshold; a negligible momentum keeps the zero request, which also
     avoids a 0/0 division */
    Eigen::Vector3f Delta_H_B = Eigen::Vector3f::Zero(); /* [Nms] net desired angular momentum change */
    const float hsMin = this->cfg.getHsMin();
    if (hs >= hsMin && hs >= kZeroMomentumTolerance) {
        Delta_H_B = (-(hs - hsMin) / hs) * hs_B;
    }

    return Delta_H_B;
}
