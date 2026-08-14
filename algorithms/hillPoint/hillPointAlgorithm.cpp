#include "hillPointAlgorithm.h"
#include "utilities/fsw/rigidBodyKinematics.hpp"
#include "utilities/fsw/safeMath.h"
#include <math.h>
#include <Eigen/Geometry>

// NOLINTBEGIN(bugprone-easily-swappable-parameters)
// bugprone-easily-swappable-parameters: the Vector3d position/velocity inputs are documented in
// the header and follow the standard (sc, planet) ordering.
HillPointOutput HillPointAlgorithm::update(const Eigen::Vector3d& r_BN_N,
                                           const Eigen::Vector3d& v_BN_N,
                                           const Eigen::Vector3d& r_PN_N,
                                           const Eigen::Vector3d& v_PN_N) {
    // Position/velocity scale work stays in double to avoid losing precision in
    // difference-of-large-numbers (e.g. heliocentric vectors) and large products
    // like orbitRadius^2.
    const Eigen::Vector3d r_BP_N = r_BN_N - r_PN_N;
    const Eigen::Vector3d v_BP_N = v_BN_N - v_PN_N;

    const double orbitRadius = r_BP_N.stableNorm();
    const Eigen::Vector3d orbitAngMomentum = r_BP_N.cross(v_BP_N);

    // Robustness threshold against divide-by-near-zero. Note the original Xmera comment claimed
    // "1 km" but the value is 1.0 in the same units as r_BN_N, which is meters.
    const bool isOrbitRadiusValid = orbitRadius > kMinOrbitRadius;
    // Guard against r/v collinearity: if r_BP_N and v_BP_N are nearly collinear, the orbit
    // normal is invalid and the Hill frame is undefined.
    const double dotProduct = r_BP_N.stableNormalized().dot(v_BP_N.stableNormalized());
    const double posVelSeparationAngle = safeAcos(fabs(dotProduct));
    const bool isPosVelSeparationValid = posVelSeparationAngle >= kSmallAngleThreshold;
    // Guard against zero relative velocity: The velocity squaredNorm check is added because eigen
    // normalizes a zero vector to zero rather than a NaN, which produces a safeAcos(0) that gives
    // a valid 90 deg.
    const bool isVelocityNonZero = v_BP_N.squaredNorm() > 0.0;

    HillPointOutput out{};  // Outputs default to zero as fallback

    if (isOrbitRadiusValid && isPosVelSeparationValid && isVelocityNonZero) {
        // Hill-frame unit vectors -- magnitude 1 by construction, so float is fine.
        const Eigen::Vector3d i_r_d = r_BP_N.stableNormalized();
        const Eigen::Vector3d i_h_d = orbitAngMomentum.stableNormalized();
        const Eigen::Vector3d i_theta_d = i_h_d.cross(i_r_d);

        // DCM from inertial frame N to Hill reference frame R, stored in float.
        Eigen::Matrix3f dcm_RN;
        dcm_RN.row(0) = i_r_d.cast<float>();
        dcm_RN.row(1) = i_theta_d.cast<float>();
        dcm_RN.row(2) = i_h_d.cast<float>();

        const double dfdt = orbitAngMomentum.stableNorm() / (orbitRadius * orbitRadius);  // true anomaly rate
        const double ddfdt2 = -2.0 * v_BP_N.dot(i_r_d) / orbitRadius * dfdt;              // true anomaly acceleration

        const Eigen::Vector3f omega_RN_R{0.0F, 0.0F, static_cast<float>(dfdt)};
        const Eigen::Vector3f domega_RN_R{0.0F, 0.0F, static_cast<float>(ddfdt2)};

        out.sigma_RN = dcmToMrp(dcm_RN);
        out.omega_RN_N = dcm_RN.transpose() * omega_RN_R;
        out.domega_RN_N = dcm_RN.transpose() * domega_RN_R;
    }
    // else: degenerate geometry (either the orbit radius is below threshold, the relative
    // velocity is exactly zero, or r_BP_N and v_BP_N are collinear) -- leave output at the
    // zero default rather than divide by ~0 or output NaNs.
    return out;
}
// NOLINTEND(bugprone-easily-swappable-parameters)
