#include "mrpRotationAlgorithm.h"
#include <architecture/utilities/rigidBodyKinematics.hpp>

/*! @brief Construct the algorithm with a validated configuration. Seeds the integrating runtime
 state (sigma_RR0, omega_RR0_R) from the configured initial values so the first update produces a
 sensible result without an explicit reset() call.
 @param config Validated configuration (initial sigma_RR0, omega_RR0_R, and integration controlPeriod).
 */
MrpRotationAlgorithm::MrpRotationAlgorithm(const MrpRotationConfig& config)
    : cfg(config), sigma_RR0(config.getInitialSigmaRR0()), omega_RR0_R(config.getOmegaRR0R()) {}

/*! @brief Replace the algorithm's stored configuration at runtime. Runtime integrator state
 (sigma_RR0, omega_RR0_R) is not re-seeded; call reset() if the new config's initial values should
 be applied.
 @param config New validated configuration to apply.
 */
void MrpRotationAlgorithm::setConfig(const MrpRotationConfig& config) { this->cfg = config; }

/*! @brief Reset the algorithm: re-seed the active sigma_RR0 / omega_RR0_R from the configured
 initial values.
 */
void MrpRotationAlgorithm::reset() {
    this->sigma_RR0 = this->cfg.getInitialSigmaRR0();
    this->omega_RR0_R = this->cfg.getOmegaRR0R();
}

/*! @brief Take the input attitude reference frame and superimpose the algorithm's MRP rotation on
 top of it, advancing sigma_RR0 one forward-Euler step (using the configured controlPeriod as dt)
 and emitting the output reference frame.
 @param attRef Guidance reference input (sigma_R0N, omega_R0N_N, domega_R0N_N), already converted to Eigen by the
 adapter.
 @return MrpRotationOutput Output reference frame R: sigma_RN, omega_RN_N, domega_RN_N.
 */
MrpRotationOutput MrpRotationAlgorithm::update(const MrpRotationAttRefInputs& attRef) {
    return this->computeMRPRotationReference(attRef.sigma_R0N, attRef.omega_R0N_N, attRef.domega_R0N_N);
}

/*! @brief Compute the reference frame (MRP attitude, angular velocity, angular acceleration)
 defined by an initial MRP set and a constant R-frame angular velocity. Performs one forward-Euler
 step on the MRP kinematic differential equation and uses the transport theorem (with R-frame
 d/dt(omega_RR0) = 0) for the inertial angular acceleration.
 @param sigma_R0N Input reference attitude as MRPs.
 @param omega_R0N_N Input reference frame angular velocity in inertial-frame components [rad/s].
 @param domega_R0N_N Input reference frame angular acceleration in inertial-frame components [rad/s^2].
 @return MrpRotationOutput The output reference frame (sigma_RN, omega_RN_N, domega_RN_N).
 */
// NOLINTBEGIN(bugprone-easily-swappable-parameters)
// sigma_R0N (attitude MRP), omega_R0N_N (angular velocity), and domega_R0N_N (angular acceleration) are physically
// distinct quantities with documented names; reordering would be caught by reviewers and tests.
MrpRotationOutput MrpRotationAlgorithm::computeMRPRotationReference(const Eigen::Vector3f& sigma_R0N,
                                                                    const Eigen::Vector3f& omega_R0N_N,
                                                                    const Eigen::Vector3f& domega_R0N_N) {
    // NOLINTEND(bugprone-easily-swappable-parameters)
    constexpr float kMrpKinematicGain = 0.25F;
    constexpr float kMrpShadowSwitchNorm = 1.0F;

    /*! - Compute attitude reference frame R/N information */
    const Eigen::Matrix3f B = bmatMrp(this->sigma_RR0);
    const Eigen::Vector3f sigmaDot_RR0 = kMrpKinematicGain * B * this->omega_RR0_R;
    const Eigen::Vector3f mrpSetNew = this->sigma_RR0 + (sigmaDot_RR0 * this->cfg.getControlPeriod());
    this->sigma_RR0 = mrpSwitch(mrpSetNew, kMrpShadowSwitchNorm);

    const Eigen::Matrix3f dcm_RR0 = mrpToDcm(this->sigma_RR0);
    const Eigen::Matrix3f dcm_R0N = mrpToDcm(sigma_R0N);
    const Eigen::Matrix3f dcm_RN = dcm_RR0 * dcm_R0N;

    const Eigen::Vector3f omega_RR0_N = dcm_RN.transpose() * this->omega_RR0_R;
    const Eigen::Vector3f domega_RR0_N = omega_R0N_N.cross(omega_RR0_N);

    const Eigen::Vector3f sigma_RN = dcmToMrp(dcm_RN);
    const Eigen::Vector3f omega_RN_N = omega_RR0_N + omega_R0N_N;
    const Eigen::Vector3f domega_RN_N = domega_RR0_N + domega_R0N_N;

    return MrpRotationOutput{
        .sigma_RN = sigma_RN,
        .omega_RN_N = omega_RN_N,
        .domega_RN_N = domega_RN_N,
    };
}
