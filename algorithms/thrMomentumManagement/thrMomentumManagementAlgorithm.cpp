/*
    Thruster RW Momentum Management

 */

#include "thrMomentumManagementAlgorithm.h"

/*! Re-arm the one-shot momentum dumping request so the next update() assesses the RW cluster momentum.
 @return void
 */
void ThrMomentumManagementAlgorithm::reInitialize() { this->dumpRequested = true; }

/*! The RW momentum level is assessed to determine if a momentum dumping maneuver is required.
 This checking only happens once after the algorithm is re-initialized.  To run this again afterwards,
 reInitialize() must be called again.
 @return std::optional<Eigen::Vector3d> [Nms] the requested momentum change, or nullopt if the check did not run
 @param wheelSpeeds [r/s] current reaction wheel speeds
 */
std::optional<Eigen::Vector3d> ThrMomentumManagementAlgorithm::update(
    const Eigen::Vector<double, kMaxNumRw>& wheelSpeeds) {
    std::optional<Eigen::Vector3d> Delta_H_B; /* [Nms]  net desired angular momentum change */

    /*! - check if a momentum dumping check has been requested */
    if (this->dumpRequested) {
        /*! - compute net RW momentum magnitude */
        Eigen::Vector3d hs_B = Eigen::Vector3d::Zero(); /* RW angular momentum */
        for (uint32_t i = 0; i < this->rwArrayConfig.numRW; i++) {
            hs_B += this->rwArrayConfig.JsList[i] * wheelSpeeds[i] * this->rwArrayConfig.GsMatrix_B.col(i);
        }
        const double hs = hs_B.norm(); /* net RW cluster angular momentum magnitude */

        /*! - check if momentum dumping is required */
        if (hs < this->hs_min) {
            /* Momentum dumping not required */
            Delta_H_B = Eigen::Vector3d::Zero();
        } else {
            Delta_H_B = (-(hs - this->hs_min) / hs) * hs_B;
        }
        this->dumpRequested = false;
    }

    return Delta_H_B;
}
