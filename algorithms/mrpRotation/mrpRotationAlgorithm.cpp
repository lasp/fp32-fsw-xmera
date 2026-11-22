#include "mrpRotationAlgorithm.h"
#include <utilities/fsw/eigenSupport.h>
#include <Eigen/Geometry>
#include <utilities/fsw/rigidBodyKinematics.hpp>

/*! @brief Construct the algorithm with a validated configuration and start the rotation at identity.
 @param config Validated configuration.
 */
MrpRotationAlgorithm::MrpRotationAlgorithm(const MrpRotationConfig& config) : cfg(config) {
    setConfig(config);
    reInitialize();
}

/*! @brief Replace the configuration at runtime. The new base frame, mountings and pointing offset apply at the
 next update; the integrated rotation sigma_RRp is kept.

 Rc is where the body must be for the measured component frame C to sit where the nominal component frame Cn sits
 when the body is on the base, so [RcRb] has the entries of [BC][CnB]. The offset and the spin axis are given in
 component axes, and [BC] turns them into body axes. The MRP of a rotation lies along its axis, so [BC] turns the
 offset MRP like a vector.
 @param config New validated configuration.
 */
void MrpRotationAlgorithm::setConfig(const MrpRotationConfig& config) {
    this->cfg = config;
    const Eigen::Matrix3f dcm_BC = mrpToDcm(this->cfg.getSigma_CB()).transpose();
    const Eigen::Matrix3f dcm_RcRb = dcm_BC * mrpToDcm(this->cfg.getSigma_CnB());
    const Eigen::Vector3f sigma_RpRc = dcm_BC * this->cfg.getSigma_RpRc_C();
    this->dcm_RpR0 = mrpToDcm(sigma_RpRc) * dcm_RcRb * mrpToDcm(this->cfg.getSigma_RbR0());
    this->spinAxis_Rp = dcm_BC * this->cfg.getSpinAxis_C();
}

/*! @brief Restart the integrated rotation sigma_RRp at identity. */
void MrpRotationAlgorithm::reInitialize() { this->sigma_RRp.setZero(); }

/*! @brief Advance sigma_RRp one forward-Euler step of controlPeriod and compose the output reference in pointing
 order, [RN] = [RRp][RpRc][RcRb][RbR0][R0N]: the base, the component, the offset, then the spin.
 @param attRef Input reference R0 (sigma_R0N, omega_R0N_N, domega_R0N_N).
 @return MrpRotationOutput Output reference R: sigma_RN, omega_RN_N, domega_RN_N.
 */
MrpRotationOutput MrpRotationAlgorithm::update(const MrpRotationAttRefInputs& attRef) {
    const Eigen::Vector3f omega_RRp_R = mrpToDcm(this->sigma_RRp) * (this->cfg.getSpinRate() * this->spinAxis_Rp);
    const Eigen::Vector3f sigmaDot_RRp = dmrp(this->sigma_RRp, omega_RRp_R);
    this->sigma_RRp = mrpSwitch(Eigen::Vector3f(this->sigma_RRp + (sigmaDot_RRp * this->cfg.getControlPeriod())));

    const Eigen::Matrix3f dcm_RRp = mrpToDcm(this->sigma_RRp);
    const Eigen::Matrix3f dcm_R0N = mrpToDcm(attRef.sigma_R0N);
    const Eigen::Matrix3f dcm_RN = dcm_RRp * this->dcm_RpR0 * dcm_R0N;

    // Rp is fixed in R0, so omega_RR0 = omega_RRp.
    const Eigen::Vector3f omega_RRp_N = dcm_RN.transpose() * omega_RRp_R;
    const Eigen::Vector3f domega_RRp_N = attRef.omega_R0N_N.cross(omega_RRp_N);

    return MrpRotationOutput{
        .sigma_RN = dcmToMrp(dcm_RN),
        .omega_RN_N = omega_RRp_N + attRef.omega_R0N_N,
        .domega_RN_N = domega_RRp_N + attRef.domega_R0N_N,
    };
}
