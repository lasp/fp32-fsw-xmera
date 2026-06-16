#include "mrpPDAlgorithm.h"

MrpPDAlgorithm::MrpPDAlgorithm(const MrpPDConfig& config) : cfg(config) { setConfig(config); }

void MrpPDAlgorithm::setConfig(const MrpPDConfig& config) { this->cfg = config; }

/*! Update method for mrpPD control algorithm. This method takes the attitude and rate errors relative to the
 reference frame, as well as the reference frame angular acceleration, and computes the required control
 torque Lr.
 @return Eigen::Vector3f
 @param sigma_BR Body to reference MRP
 @param omega_BR_B Body to reference rate
 @param domega_RN_B Body to reference acceleration
*/
Eigen::Vector3f MrpPDAlgorithm::update(const Eigen::Vector3f& sigma_BR,
                                       const Eigen::Vector3f& omega_BR_B,
                                       const Eigen::Vector3f& domega_RN_B) const {
    // Compute required attitude control torque vector
    const Eigen::Vector3f Lr = -this->cfg.getProportionalGainK() * sigma_BR -
                               this->cfg.getDerivativeGainP() * omega_BR_B + this->cfg.getInertia() * domega_RN_B -
                               this->cfg.getKnownTorquePntB_B();  // [Nm]

    return Lr;
}
