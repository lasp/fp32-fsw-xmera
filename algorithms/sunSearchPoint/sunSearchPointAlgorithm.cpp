#include "sunSearchPointAlgorithm.h"

#include "utilities/fsw/rigidBodyKinematics.hpp"
#include "utilities/fsw/safeMath.h"
#include "utilities/fsw/timeConstants.h"

#include <Eigen/Geometry>
#include <numbers>

SunSearchPointAlgorithm::SunSearchPointAlgorithm(const SunSearchPointConfig& config) : cfg(config) {
    this->setConfig(config);
    this->reInitialize();
}

/*! Setter for the sun-search rotation sequence configuration. Installs the parameters only; call
 reset() to re-arm the search phase.
 @return void
 @param config Validated rotation-sequence configuration
*/
void SunSearchPointAlgorithm::setConfig(const SunSearchPointConfig& config) {
    this->cfg = config;
    this->controlPeriodNs = static_cast<uint64_t>(static_cast<double>(config.getControlPeriod()) * kSec2Nano);
    this->precomputeEndTimes();
}

/*! Precompute the cumulative end time (in nanoseconds) of each rotation in the search sequence. */
void SunSearchPointAlgorithm::precomputeEndTimes() {
    uint64_t cumulativeEndTimeNs = 0U;
    for (uint32_t i = 0U; i < kNumSunSearchRotations; ++i) {
        cumulativeEndTimeNs +=
            static_cast<uint64_t>(static_cast<double>(this->cfg.getRotations().at(i).rotationDuration) * kSec2Nano);
        this->rotationEndTimes.at(i) = cumulativeEndTimeNs;
    }
}

/*! Re-arms the runtime state machine so the search timeline restarts from zero and the phase begins
 again in search. Does not touch configuration.
 @return void
*/
void SunSearchPointAlgorithm::reInitialize() {
    this->elapsedTimeNs = 0U;
    this->phase = Phase::Searching;
    this->searchFailed = false;
}

/*! Update method for the sunSearchPoint guidance algorithm. Runs the sun-search rotation sequence
 until the sun is acquired (observation count reaches the threshold after the first rotation) or
 the sequence elapses, then performs closed-loop sun pointing.
 @return SunSearchPointOutput Attitude guidance output
 @param rHat_SB_B Sun direction vector in body frame
 @param omega_BN_B Body angular velocity vector
 @param numCssViewingSun Number of valid coarse-sun-sensor observations this cycle
*/
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
SunSearchPointOutput SunSearchPointAlgorithm::update(const Eigen::Vector3f& rHat_SB_B,
                                                     const Eigen::Vector3f& omega_BN_B,
                                                     const int numCssViewingSun) {
    // Evaluate the one-way SEARCH -> POINT transition. The first rotation always runs to
    // completion; after that, a sufficient observation count transitions to pointing, and the
    // full sequence elapsing forces the transition regardless of observations.
    if (this->phase == Phase::Searching) {
        const bool firstRotationComplete = this->elapsedTimeNs >= this->rotationEndTimes.at(0);
        const bool sequenceComplete = this->elapsedTimeNs >= this->rotationEndTimes.at(kNumSunSearchRotations - 1U);
        const bool sunAcquired = firstRotationComplete && (numCssViewingSun >= this->cfg.getObservationThreshold());
        if (sunAcquired) {
            this->phase = Phase::Pointing;
        } else if (sequenceComplete) {
            // Full sequence elapsed without acquiring the sun: the search failed. Latch the fault.
            this->phase = Phase::Pointing;
            this->searchFailed = true;
        }
    }

    SunSearchPointOutput output = (this->phase == Phase::Pointing) ? this->computePointing(rHat_SB_B, omega_BN_B)
                                                                   : this->computeSearch(omega_BN_B);
    output.faultDetected = this->searchFailed;

    // Advance the search timeline by one control period for the next update() call.
    this->elapsedTimeNs += this->controlPeriodNs;
    return output;
}

/*! Compute the search-phase guidance: a constant reference rate about the active rotation's body
 axis and zero attitude error. */
SunSearchPointOutput SunSearchPointAlgorithm::computeSearch(const Eigen::Vector3f& omega_BN_B) const {
    // Select the first rotation whose cumulative end time has not yet been reached; hold the last
    // rotation once the sequence has finished (only reachable transiently before the transition).
    uint32_t activeIndex = kNumSunSearchRotations - 1U;
    for (uint32_t rotationIndex = 0U; rotationIndex < kNumSunSearchRotations; ++rotationIndex) {
        if (this->elapsedTimeNs < this->rotationEndTimes.at(rotationIndex)) {
            activeIndex = rotationIndex;
            break;
        }
    }
    const RotationProperties& rot = this->cfg.getRotations().at(activeIndex);
    const Eigen::Vector3f omega_RN_B_search =
        Eigen::Vector3f::Unit(static_cast<Eigen::Index>(rot.rotationAxis)) * rot.rotationRate;

    return SunSearchPointOutput{.sigma_BR = Eigen::Vector3f::Zero(),
                                .omega_BR_B = omega_BN_B - omega_RN_B_search,
                                .omega_RN_B = omega_RN_B_search};
}

/*! Compute the pointing-phase guidance: the attitude/attitude-rate errors that align the commanded
 body axis with the sun heading, or the configured fallback rate if no sun direction is available. */
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
SunSearchPointOutput SunSearchPointAlgorithm::computePointing(const Eigen::Vector3f& rHat_SB_B,
                                                              const Eigen::Vector3f& omega_BN_B) const {
    SunSearchPointOutput output{};

    const Eigen::Vector3f rHatNormalized_SB_B = rHat_SB_B.stableNormalized();

    // Computing the attitude guidance states sigma_BR and omega_RN_B if valid sun direction is available
    if (rHatNormalized_SB_B.stableNorm() > 0.0F) {
        // Compute the current sun angle error
        float const sunAngleErr = safeAcosf(this->cfg.getSHatBdyCmd().dot(rHatNormalized_SB_B));

        // Compute the heading error relative to the sun direction vector
        Eigen::Vector3f e_axis{};  // Eigen Axis
        constexpr float kSmallAngle = 1e-3F;
        // The commanded body vector is nearly opposite the sun heading
        if (static_cast<float>(std::numbers::pi) - sunAngleErr < kSmallAngle) {
            e_axis = this->cfg.getSHatBdyCmd().unitOrthogonal();  // find orthogonal unit vector to sHatBdyCmd
            // Normal case where sun and commanded body vectors are not aligned
        } else {
            e_axis = rHatNormalized_SB_B.cross(this->cfg.getSHatBdyCmd());
        }
        Eigen::Vector3f const e_hat = e_axis.stableNormalized();
        Eigen::Vector3f sigma_BR = safeTanf(sunAngleErr * 0.25F) * e_hat;
        sigma_BR = mrpSwitch(sigma_BR);

        output.sigma_BR = sigma_BR;
        // Rate tracking error is the body rate to bring spacecraft to rest
        output.omega_RN_B = this->cfg.getSunAxisSpinRate() * rHatNormalized_SB_B;
    } else {
        output.sigma_BR = Eigen::Vector3f::Zero();
        output.omega_RN_B = this->cfg.getOmega_RN_B();
    }

    // Compute the hub angular rate error omega_BR_B
    output.omega_BR_B = omega_BN_B - output.omega_RN_B;

    return output;
}
