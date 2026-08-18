/*
    Thruster RW Momentum Management

 */

#include "thrMomentumManagementAlgorithm.h"

// [Nms] net RW momentum below this magnitude is treated as zero.
constexpr float kZeroMomentumTolerance = 1e-6F;

/*! Construct the algorithm from a validated configuration and seed the integrator state.
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

/*! Re-seed the runtime integrator state (the excess-momentum integral and its previous sample).
 @return void
 */
void ThrMomentumManagementAlgorithm::reInitialize() {
    this->hsInt_B.setZero();
    this->priorHsExcess_B.setZero();
}

/*! The RW momentum level is assessed on every call to determine the torque that dumps the momentum held above
 the hsMin threshold. The integral term accumulates across calls, so this advances the integrator state.
 @return Eigen::Vector3f [Nm] the requested body-frame torque
 @param wheelSpeeds [r/s] current reaction wheel speeds
 */
Eigen::Vector3f ThrMomentumManagementAlgorithm::update(const Eigen::Vector<float, kMaxNumRw>& wheelSpeeds) {
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

    /*! - advance the trapezoidal integral of the excess momentum, using the fixed control period as the step.
     Integrating the excess rather than the raw momentum keeps the integral from winding up inside the deadband */
    this->hsInt_B += 0.5F * params.controlPeriod * (this->priorHsExcess_B + hsExcess_B);
    this->priorHsExcess_B = hsExcess_B;

    /*! - anti-windup: clamp each integral component to the configured limit, preserving its sign */
    for (Eigen::Index i = 0; i < 3; ++i) {
        const float magnitude = fabsf(this->hsInt_B(i));
        if (magnitude > params.integralLimit) {
            this->hsInt_B(i) *= params.integralLimit / magnitude;
        }
    }

    /*! - the requested torque opposes the excess momentum and its accumulation */
    return -params.K * hsExcess_B - params.Ki * this->hsInt_B;
}
