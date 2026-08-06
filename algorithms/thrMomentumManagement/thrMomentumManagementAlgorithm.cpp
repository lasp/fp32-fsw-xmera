/*
    Thruster RW Momentum Management

 */

#include "thrMomentumManagementAlgorithm.h"

// [Nms] net RW momentum below this magnitude is treated as zero. Only reachable when hs_min is itself
// zero, since any larger threshold makes the hs < hs_min branch short-circuit first.
constexpr float kZeroMomentumTolerance = 1e-6F;

/*! Construct the algorithm from a validated configuration and arm the one-shot momentum dumping request.
 @param config the validated configuration
 */
ThrMomentumManagementAlgorithm::ThrMomentumManagementAlgorithm(const ThrMomentumManagementConfig& config)
    : cfg(config) {
    this->setConfig(config);
    this->reInitialize();
}

/*! Install the validated configuration. Runtime state is untouched; use reInitialize() for that.
 @return void
 @param config the validated configuration
 */
void ThrMomentumManagementAlgorithm::setConfig(const ThrMomentumManagementConfig& config) { this->cfg = config; }

/*! Re-arm the one-shot momentum dumping request so the next update() assesses the RW cluster momentum.
 @return void
 */
void ThrMomentumManagementAlgorithm::reInitialize() { this->dumpRequested = true; }

/*! The RW momentum level is assessed to determine if a momentum dumping maneuver is required.
 This checking only happens once after the algorithm is re-initialized.  To run this again afterwards,
 reInitialize() must be called again.
 @return std::optional<Eigen::Vector3f> [Nms] the requested momentum change, or nullopt if the check did not run
 @param wheelSpeeds [r/s] current reaction wheel speeds
 */
std::optional<Eigen::Vector3f> ThrMomentumManagementAlgorithm::update(
    const Eigen::Vector<float, kMaxNumRw>& wheelSpeeds) {
    std::optional<Eigen::Vector3f> Delta_H_B; /* [Nms]  net desired angular momentum change */

    /*! - check if a momentum dumping check has been requested */
    if (this->dumpRequested) {
        /*! - compute net RW momentum magnitude */
        const ThrMomentumManagementRwArrayConfiguration& rwArrayConfig = this->cfg.getRwArrayConfiguration();
        Eigen::Vector3f hs_B = Eigen::Vector3f::Zero(); /* RW angular momentum */
        for (uint32_t i = 0; i < rwArrayConfig.numRW; i++) {
            hs_B += rwArrayConfig.JsList(i) * wheelSpeeds(i) * rwArrayConfig.GsMatrix_B.col(i);
        }
        const float hs = hs_B.norm(); /* net RW cluster angular momentum magnitude */

        /*! - check if momentum dumping is required */
        const float hsMin = this->cfg.getHsMin();
        if (hs < hsMin || hs < kZeroMomentumTolerance) {
            /* Momentum dumping not required */
            Delta_H_B = Eigen::Vector3f::Zero();
        } else {
            Delta_H_B = (-(hs - hsMin) / hs) * hs_B;
        }
        this->dumpRequested = false;
    }

    return Delta_H_B;
}
