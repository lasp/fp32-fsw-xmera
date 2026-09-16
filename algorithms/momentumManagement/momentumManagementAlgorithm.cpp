/*
    Thruster RW Momentum Management

 */

#include "momentumManagementAlgorithm.h"

/*! Construct the algorithm from a validated configuration and seed the integrator state.
 @param config the validated configuration
 */
MomentumManagementAlgorithm::MomentumManagementAlgorithm(const MomentumManagementConfig& config) : cfg(config) {
    this->setConfig(config);
    this->reInitialize();
}

/*! Install the validated configuration. Runtime state is untouched; use reInitialize() for that.
 @return void
 @param config the validated configuration
 */
void MomentumManagementAlgorithm::setConfig(const MomentumManagementConfig& config) { this->cfg = config; }

/*! Re-seed the runtime integrator state (the momentum integral and its previous sample).
 @return void
 */
void MomentumManagementAlgorithm::reInitialize() {
    this->hsInt_B.setZero();
    this->priorHs_B.setZero();
}

/*! The RW momentum level is assessed on every call to determine the torque that dumps it. The integral term
 accumulates across calls, so this advances the integrator state.
 @return Eigen::Vector3f [Nm] the requested body-frame torque
 @param wheelSpeeds [r/s] current reaction wheel speeds
 */
Eigen::Vector3f MomentumManagementAlgorithm::update(const Eigen::Vector<float, kMaxNumRw>& wheelSpeeds) {
    /*! - compute the net RW momentum and its magnitude */
    const MomentumManagementRwArrayConfiguration& rwArrayConfig = this->cfg.getRwArrayConfiguration();
    Eigen::Vector3f hs_B = Eigen::Vector3f::Zero(); /* RW angular momentum */
    for (uint32_t i = 0; i < rwArrayConfig.numRW; ++i) {
        hs_B += rwArrayConfig.JsList(i) * wheelSpeeds(i) * rwArrayConfig.GsMatrix_B.col(i);
    }
    const float hsNorm = hs_B.stableNorm(); /* net RW cluster angular momentum magnitude */

    const MomentumManagementControlParameters& params = this->cfg.getControlParameters();
    Eigen::Vector3f Lr_B = Eigen::Vector3f::Zero(); /* [Nm] requested body-frame torque */

    /*! - the threshold is a deadband on the momentum the law acts on: at or above it the whole cluster momentum
     is dumped, below it nothing is dumped and nothing enters the integral */
    if (hsNorm >= params.hsMin) {
        /*! - advance the trapezoidal integral of the cluster momentum, using the fixed control period as the
         step */
        this->hsInt_B += 0.5F * params.controlPeriod * (this->priorHs_B + hs_B);

        /*! - anti-windup: clamp each integral component to the configured limit, preserving its sign */
        for (Eigen::Index i = 0; i < 3; ++i) {
            const float magnitude = fabsf(this->hsInt_B(i));
            if (magnitude > params.integralLimit) {
                this->hsInt_B(i) *= params.integralLimit / magnitude;
            }
        }

        this->priorHs_B = hs_B;

        /*! - the requested torque opposes the stored momentum and its accumulation */
        Lr_B = -params.K * hs_B - params.Ki * this->hsInt_B;
    } else {
        /*! - inside the deadband the integral holds what it already carries, and goes on driving the request */
        this->priorHs_B.setZero();
        Lr_B = -params.Ki * this->hsInt_B;
    }

    return Lr_B;
}
