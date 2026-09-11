#include "thrustVectoringAlgorithm.h"

#include <math.h>

#include "utilities/fsw/safeMath.h"
#include <Eigen/Geometry>

namespace {
/*! Solve for the thrust direction that produces the requested torque Lreq about the center of mass, for a thrust
 of the given magnitude acting through the point M that r_MC is measured from.
 @return the thrust unit direction, in the frame the arguments were given in
*/
Eigen::Vector3f solveThrustDirection(const Eigen::Vector3f& r_MC, float thrust, const Eigen::Vector3f& Lreq) {
    const float b = r_MC.norm();  // moment arm about M; the configuration guarantees b > kMinR_CM
    const Eigen::Vector3f rHat_MC = r_MC / b;

    // (1) Invert |L| = thrust * b * |tPerp| for the requested torque. The cross product also discards the
    //     component of the request along r_MC, which no orientation can produce.
    const Eigen::Vector3f tPerpRequested = (Lreq / (thrust * b)).cross(rHat_MC);

    // (2) Its length is the delivered torque as a fraction of the largest available, so a magnitude above one asks
    //     for more than thrust * b; limiting it there saturates at the maximum in the requested direction.
    const float tPerpMagnitude = fminf(tPerpRequested.stableNorm(), 1.0F);
    const Eigen::Vector3f tPerp = tPerpMagnitude * tPerpRequested.stableNormalized();

    // (3) Compute the component along r_MC, exactly zero once saturated. Both signs deliver the same torque, since
    //     this component produces no torque. Take the one that fires the thrust from M towards the center of
    //     mass, which keeps the thrust acting on the vehicle from outside it.
    const float tAlongMagnitude = safeSqrtf(1.0F - (tPerpMagnitude * tPerpMagnitude));
    const Eigen::Vector3f tAlong = -tAlongMagnitude * rHat_MC;

    const Eigen::Vector3f tHat = (tPerp + tAlong).stableNormalized();

    return tHat;
}

}  // namespace

/*! @brief Construct the algorithm with a validated configuration.
 @param config Validated configuration (thrust point, thrust magnitude and center of mass).
*/
ThrustVectoringAlgorithm::ThrustVectoringAlgorithm(const ThrustVectoringConfig& config) : cfg(config) {
    this->setConfig(config);
}

/*! @brief Replace the stored configuration at runtime.
 @param config New validated configuration to apply.
*/
void ThrustVectoringAlgorithm::setConfig(const ThrustVectoringConfig& config) { this->cfg = config; }

/*! This method computes the thrust direction that produces the requested torque about the center of mass. A zero
 request puts the line of action through the center of mass, which produces no torque.
 @return [-] thrust unit direction, body frame
 @param Lreq_B [Nm] requested torque about the center of mass, body frame
*/
Eigen::Vector3f ThrustVectoringAlgorithm::update(const Eigen::Vector3f& Lreq_B) const {
    const Eigen::Vector3f r_MC_B = this->cfg.getR_MB_B() - this->cfg.getR_CB_B();

    return solveThrustDirection(r_MC_B, this->cfg.getThrust(), Lreq_B);
}
