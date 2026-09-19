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
    this->priorHsDumpable_B.setZero();
}

/*! The RW momentum level is assessed on every call to determine the torque that dumps it. The integral term
 accumulates across calls, so this advances the integrator state.
 @return Eigen::Vector3f [Nm] the requested body-frame torque
 @param wheelSpeeds [r/s] current reaction wheel speeds
 */
Eigen::Vector3f MomentumManagementAlgorithm::update(const Eigen::Vector<float, kMaxNumRw>& wheelSpeeds) {
    /*! - compute the net RW momentum */
    const MomentumManagementRwArrayConfiguration& rwArrayConfig = this->cfg.getRwArrayConfiguration();
    Eigen::Vector3f hs_B = Eigen::Vector3f::Zero(); /* RW angular momentum */
    for (uint32_t i = 0; i < kMaxNumRw; ++i) {
        hs_B += rwArrayConfig.JsList(i) * wheelSpeeds(i) * rwArrayConfig.GsMatrix_B.col(i);
    }

    /*! - keep only the momentum the effectors can dump. Momentum they cannot remove must not reach the law: it
     would hold the deadband open and wind the integral up along a direction no torque can act on */
    const MomentumManagementControlParameters& params = this->cfg.getControlParameters();
    const Eigen::Vector3f hsDumpable_B = params.dumpableProjection_B * hs_B; /* [Nms] dumpable RW momentum */
    const float hsNorm = hsDumpable_B.stableNorm();                          /* dumpable momentum magnitude */

    Eigen::Vector3f Lr_B = Eigen::Vector3f::Zero(); /* [Nm] requested body-frame torque */

    /*! - the threshold gates the whole law: at or above it the entire dumpable momentum is dumped, below it
     the module requests nothing */
    if (hsNorm >= params.hsMin) {
        /*! - advance the trapezoidal integral of the dumpable momentum, using the fixed control period as the
         step */
        this->hsInt_B += 0.5F * params.controlPeriod * (this->priorHsDumpable_B + hsDumpable_B);

        /*! - anti-windup: clamp each integral component to the configured limit, preserving its sign */
        for (Eigen::Index i = 0; i < 3; ++i) {
            const float magnitude = fabsf(this->hsInt_B(i));
            if (magnitude > params.integralLimit) {
                this->hsInt_B(i) *= params.integralLimit / magnitude;
            }
        }

        this->priorHsDumpable_B = hsDumpable_B;

        /*! - the requested torque opposes the dumpable momentum and its accumulation */
        Lr_B = -params.K * hsDumpable_B - params.Ki * this->hsInt_B;
    } else {
        /*! - below the threshold the dump is over: request no torque and clear the integrator, so the next
         dump starts from zero instead of carrying an impulse deficit that no longer applies */
        this->hsInt_B.setZero();
        this->priorHsDumpable_B.setZero();
    }

    return Lr_B;
}
