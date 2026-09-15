#include "thrustVectoringAlgorithm.h"

#include <math.h>

#include "utilities/fsw/safeMath.h"
#include <Eigen/Geometry>

/*! @brief Construct the algorithm with a validated configuration.
 @param config Validated configuration (thrust point, thrust magnitude and center of mass).
*/
ThrustVectoringAlgorithm::ThrustVectoringAlgorithm(const ThrustVectoringConfig& config) : cfg(config) {
    this->setConfig(config);
}

/*! @brief Replace the stored configuration at runtime.
 @param config New validated configuration to apply.
*/
void ThrustVectoringAlgorithm::setConfig(const ThrustVectoringConfig& config) {
    this->cfg = config;

    // The configuration guarantees a moment arm longer than kMinR_CM, so both quantities are well defined.
    const Eigen::Vector3f r_MC_B = config.getR_MB_B() - config.getR_CB_B();
    this->rHat_MC_B = r_MC_B.stableNormalized();
    this->maxAchievableTorque = config.getThrust() * r_MC_B.stableNorm();
}

/*! This method computes the thrust direction that produces the requested torque about the center of mass. A zero
 request puts the line of action through the center of mass, which produces no torque.
 @return [-] thrust unit direction, body frame
 @param Lreq_B [Nm] requested torque about the center of mass, body frame
*/
Eigen::Vector3f ThrustVectoringAlgorithm::update(const Eigen::Vector3f& Lreq_B) const {
    // (1) Only the component of the thrust perpendicular to r_MC_B produces torque. Crossing the request with
    //     rHat_MC_B turns it into that component, and discards the part of the request along r_MC_B, which no
    //     direction can produce.
    const Eigen::Vector3f tPerpRequested_B = Lreq_B.cross(this->rHat_MC_B);

    // (2) Taken as a fraction of maxAchievableTorque, so a value above one asks for more torque than the
    //     geometry can deliver; limiting it there saturates at the maximum in the requested direction.
    const float tPerpMagnitude = fminf(tPerpRequested_B.stableNorm() / this->maxAchievableTorque, 1.0F);
    const Eigen::Vector3f tPerp_B = tPerpMagnitude * tPerpRequested_B.stableNormalized();

    // (3) The rest of the unit direction goes along r_MC_B, and is exactly zero once saturated. Both signs deliver
    //     the same torque, since this component produces none. Take the one that fires the thrust from M towards
    //     the center of mass, which keeps the thrust acting on the vehicle from outside it.
    const float tAlongMagnitude = safeSqrtf(1.0F - (tPerpMagnitude * tPerpMagnitude));
    const Eigen::Vector3f tAlong_B = -tAlongMagnitude * this->rHat_MC_B;

    // The two components are perpendicular and their lengths square to one, so the sum is already a unit vector.
    // Normalizing only removes the rounding the two components carry.
    return (tPerp_B + tAlong_B).stableNormalized();
}
