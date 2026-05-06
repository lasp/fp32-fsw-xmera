#include "mrpRotationAlgorithm.h"
#include "utilities/timeConstants.h"
#include <architecture/utilities/eigenSupport.h>
#include <architecture/utilities/rigidBodyKinematics.hpp>

/*! @brief Construct the algorithm with a validated configuration. Seeds the integrating runtime
 state (sigma_RR0, omega_RR0_R) from the configured initial values so the first update produces a
 sensible result without an explicit reset() call.
 @param config Validated configuration (initial sigma_RR0, omega_RR0_R, and the dynamic-reference flag).
 */
MrpRotationAlgorithm::MrpRotationAlgorithm(const MrpRotationConfig& config)
    : cfg(config), sigma_RR0(config.getInitialSigmaRR0()), omega_RR0_R(config.getOmegaRR0R()) {}

/*! @brief Replace the algorithm's stored configuration at runtime. Runtime integrator state
 (sigma_RR0, omega_RR0_R) is not re-seeded; call reset() if the new config's initial values should
 be applied.
 @param config New validated configuration to apply.
 */
void MrpRotationAlgorithm::setConfig(const MrpRotationConfig& config) { this->cfg = config; }

/*! @brief Reset the algorithm: clear the integration timing state, the prior-command latches, and
 re-seed the active sigma_RR0 / omega_RR0_R from the configured initial values.
 */
void MrpRotationAlgorithm::reset() {
    this->priorTime = 0;
    this->priorCmdSet = Eigen::Vector3f::Zero();
    this->priorCmdRates = Eigen::Vector3f::Zero();
    this->sigma_RR0 = this->cfg.getInitialSigmaRR0();
    this->omega_RR0_R = this->cfg.getOmegaRR0R();
}

/*! @brief Take the input attitude reference frame and superimpose the algorithm's MRP rotation on
 top of it, advancing sigma_RR0 one Euler step and emitting the output reference frame.
 @param callTime The clock time at which the function was called (nanoseconds).
 @param inputRef Guidance reference input message (sigma_R0N, omega_R0N_N, domega_R0N_N).
 @param attStates Optional commanded MRP set / angular velocity, consumed only when the configured
                  dynamicReferenceEnabled flag is true.
 @return AttRefMsgF32Payload Output reference frame R: sigma_RN, omega_RN_N, domega_RN_N.
 */
AttRefMsgF32Payload MrpRotationAlgorithm::update(const uint64_t callTime,
                                                 const AttRefMsgF32Payload inputRef,
                                                 const AttStateMsgF32Payload attStates) {
    /*! - Check if a desired attitude configuration message exists. This allows for dynamic changes to the desired MRP
     * rotation */
    if (this->cfg.getDynamicReferenceEnabled()) {
        /* - Save commanded MRP set and body rates */
        this->cmdSet = Eigen::Map<const Eigen::Vector3f>(attStates.state);
        this->cmdRates = Eigen::Map<const Eigen::Vector3f>(attStates.rate);
        /* - Check the command is new */
        this->checkRasterCommands();
    }

    /*! - Compute time step to use in the integration downstream */
    this->computeTimeStep(callTime);

    const Eigen::Vector3f sigma_RN = Eigen::Map<const Eigen::Vector3f>(inputRef.sigma_RN);
    const Eigen::Vector3f omega_RN_N = Eigen::Map<const Eigen::Vector3f>(inputRef.omega_RN_N);
    const Eigen::Vector3f domega_RN_N = Eigen::Map<const Eigen::Vector3f>(inputRef.domega_RN_N);

    /*! - Compute output reference frame */
    const AttRefMsgF32Payload attRefOut = this->computeMRPRotationReference(sigma_RN, omega_RN_N, domega_RN_N);

    /*! - Update last time the module was called to current call time */
    this->priorTime = callTime;

    return attRefOut;
}

/*! @brief Detect a change in the commanded raster MRP set / rate (componentwise abs >
 kCmdChangeTolerance) and, when found, latch the new command into the integrator state
 (sigma_RR0, omega_RR0_R) and the prior-command latches.
 */
void MrpRotationAlgorithm::checkRasterCommands() {
    constexpr float kCmdChangeTolerance = 1e-6F;
    const bool prevCmdActive = ((this->cmdSet - this->priorCmdSet).array().abs() < kCmdChangeTolerance).all() &&
                               ((this->cmdRates - this->priorCmdRates).array().abs() < kCmdChangeTolerance).all();

    /*! - check if a new attitude reference command message content is availble */
    if (!prevCmdActive) {
        /*! - copy over the commanded initial MRP and rate information */
        this->sigma_RR0 = this->cmdSet;
        this->omega_RR0_R = this->cmdRates;

        /*! - reset the prior commanded attitude state variables */
        this->priorCmdSet = this->cmdSet;
        this->priorCmdRates = this->cmdRates;
    }
}

/*! @brief Derive the integration time step dt from callTime - priorTime. On the first call after
 reset() priorTime is 0, forcing dt to 0 so the integrator does not advance until a second sample
 is available.
 @param callTime The clock time at which the function was called (nanoseconds).
*/
void MrpRotationAlgorithm::computeTimeStep(const uint64_t callTime) {
    if (this->priorTime == 0) {
        this->dt = 0.0F;
    } else {
        this->dt = static_cast<float>(callTime - this->priorTime) * kNano2SecF;
    }
}

/*! @brief Compute the reference frame (MRP attitude, angular velocity, angular acceleration)
 defined by an initial MRP set and a constant R-frame angular velocity. Performs one forward-Euler
 step on the MRP kinematic differential equation and uses the transport theorem (with R-frame
 d/dt(omega_RR0) = 0) for the inertial angular acceleration.
 @param sigma_R0N Input reference attitude as MRPs.
 @param omega_R0N_N Input reference frame angular velocity in inertial-frame components [rad/s].
 @param domega_R0N_N Input reference frame angular acceleration in inertial-frame components [rad/s^2].
 @return AttRefMsgF32Payload The output reference frame (sigma_RN, omega_RN_N, domega_RN_N).
 */
// NOLINTBEGIN(bugprone-easily-swappable-parameters)
// sigma_R0N (attitude MRP), omega_R0N_N (angular velocity), and domega_R0N_N (angular acceleration) are physically
// distinct quantities with documented names; reordering would be caught by reviewers and tests.
AttRefMsgF32Payload MrpRotationAlgorithm::computeMRPRotationReference(const Eigen::Vector3f& sigma_R0N,
                                                                      const Eigen::Vector3f& omega_R0N_N,
                                                                      const Eigen::Vector3f& domega_R0N_N) {
    // NOLINTEND(bugprone-easily-swappable-parameters)
    constexpr float kMrpKinematicGain = 0.25F;
    constexpr float kMrpShadowSwitchNorm = 1.0F;

    /*! - Compute attitude reference frame R/N information */
    const Eigen::Matrix3f B = bmatMrp(this->sigma_RR0);
    const Eigen::Vector3f sigmaDot_RR0 = kMrpKinematicGain * B * this->omega_RR0_R;
    const Eigen::Vector3f mrpSetNew = this->sigma_RR0 + sigmaDot_RR0 * this->dt;
    this->sigma_RR0 = mrpSwitch(mrpSetNew, kMrpShadowSwitchNorm);
    const Eigen::Matrix3f dcm_RR0 = mrpToDcm(this->sigma_RR0);
    const Eigen::Matrix3f dcm_R0N = mrpToDcm(sigma_R0N);
    const Eigen::Matrix3f dcm_RN = dcm_RR0 * dcm_R0N;

    const Eigen::Vector3f sigma_RN = dcmToMrp(dcm_RN);

    const Eigen::Vector3f omega_RR0_N = dcm_RN.transpose() * this->omega_RR0_R;
    const Eigen::Vector3f omega_RN_N = omega_RR0_N + omega_R0N_N;

    const Eigen::Vector3f domega_RR0_N = omega_R0N_N.cross(omega_RR0_N);
    const Eigen::Vector3f domega_RN_N = domega_RR0_N + domega_R0N_N;

    AttRefMsgF32Payload attRefOut{};

    eigenVectorToCArray(sigma_RN, attRefOut.sigma_RN);
    eigenVectorToCArray(omega_RN_N, attRefOut.omega_RN_N);
    eigenVectorToCArray(domega_RN_N, attRefOut.domega_RN_N);

    return attRefOut;
}
