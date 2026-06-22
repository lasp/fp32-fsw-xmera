#include "mrpRotationAlgorithm.h"
#include <utilities/fsw/eigenSupport.h>
#include <utilities/fsw/rigidBodyKinematics.hpp>

/*! @brief Construct the algorithm with a validated configuration. Seeds the integrating runtime
 state (sigma_RR0, omega_RR0_R) from the configured initial values so the first update produces a
 sensible result.
 @param config Validated configuration (initial sigma_RR0, omega_RR0_R, and integration controlPeriod).
 */
MrpRotationAlgorithm::MrpRotationAlgorithm(const MrpRotationConfig& config)
    : cfg(config), sigma_RR0(config.getInitialSigmaRR0()), omega_RR0_R(config.getOmegaRR0R()) {}

/*! @brief Replace the algorithm's stored configuration at runtime and re-seed the runtime integrator
 state (sigma_RR0, omega_RR0_R) from the new configuration's initial values, so every reconfiguration
 restarts the rotating reference from its configured seed.
 @param config New validated configuration to apply.
 */
void MrpRotationAlgorithm::setConfig(const MrpRotationConfig& config) {
    this->cfg = config;
    this->sigma_RR0 = this->cfg.getInitialSigmaRR0();
    this->omega_RR0_R = this->cfg.getOmegaRR0R();
}

/*! @brief Take the input attitude reference frame and superimpose the algorithm's MRP rotation on
 top of it: advance sigma_RR0 by one forward-Euler step (using the configured controlPeriod as dt),
 compose with the input reference attitude/rate/acceleration, and emit the output reference frame.
 Uses the transport theorem (with R-frame d/dt(omega_RR0) = 0) for the inertial angular acceleration.
 @param attRef Guidance reference input (sigma_R0N, omega_R0N_N, domega_R0N_N), already converted to
        Eigen by the adapter.
 @return MrpRotationOutput Output reference frame R: sigma_RN, omega_RN_N, domega_RN_N.
 */
MrpRotationOutput MrpRotationAlgorithm::update(const MrpRotationAttRefInputs& attRef) {
    constexpr float kMrpKinematicGain = 0.25F;
    constexpr float kMrpShadowSwitchNorm = 1.0F;

    /*! - Advance sigma_RR0 one forward-Euler step using the MRP kinematic differential equation,
     *    then shadow-switch to keep the representation bounded. */
    const Eigen::Matrix3f B = bmatMrp(this->sigma_RR0);
    const Eigen::Vector3f sigmaDot_RR0 = kMrpKinematicGain * B * this->omega_RR0_R;
    const Eigen::Vector3f mrpSetNew = this->sigma_RR0 + (sigmaDot_RR0 * this->cfg.getControlPeriod());
    this->sigma_RR0 = mrpSwitch(mrpSetNew, kMrpShadowSwitchNorm);

    /*! - Compose with the input reference frame to produce the output R/N attitude and rates. */
    const Eigen::Matrix3f dcm_RR0 = mrpToDcm(this->sigma_RR0);
    const Eigen::Matrix3f dcm_R0N = mrpToDcm(attRef.sigma_R0N);
    const Eigen::Matrix3f dcm_RN = dcm_RR0 * dcm_R0N;

    const Eigen::Vector3f omega_RR0_N = dcm_RN.transpose() * this->omega_RR0_R;
    const Eigen::Vector3f domega_RR0_N = attRef.omega_R0N_N.cross(omega_RR0_N);

    const Eigen::Vector3f sigma_RN = dcmToMrp(dcm_RN);
    const Eigen::Vector3f omega_RN_N = omega_RR0_N + attRef.omega_R0N_N;
    const Eigen::Vector3f domega_RN_N = domega_RR0_N + attRef.domega_R0N_N;

    return MrpRotationOutput{
        .sigma_RN = sigma_RN,
        .omega_RN_N = omega_RN_N,
        .domega_RN_N = domega_RN_N,
    };
}
