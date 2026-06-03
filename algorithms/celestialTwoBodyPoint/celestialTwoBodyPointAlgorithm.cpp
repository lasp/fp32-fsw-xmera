#include "celestialTwoBodyPointAlgorithm.h"
#include "utilities/fsw/rigidBodyKinematics.hpp"
#include "utilities/fsw/safeMath.h"
#include <math.h>
#include <numbers>

CelestialTwoBodyPointAlgorithm::CelestialTwoBodyPointAlgorithm(const CelestialTwoBodyPointConfig &config)
    : cfg(config) {
    setConfig(config);
}

void CelestialTwoBodyPointAlgorithm::setConfig(const CelestialTwoBodyPointConfig &config) { this->cfg = config; }

/*! This method computes the attitude reference that points the primary axis at the primary
 celestial body while aligning a second axis as close as possible toward the secondary
 celestial body. It generates the commanded attitude and assumes that the control errors are
 computed downstream.
 @param r_celBody_N [m] primary celestial body inertial position
 @param v_celBody_N [m/s] primary celestial body inertial velocity
 @param r_secCelBody_N [m] secondary celestial body inertial position (ignored when not linked)
 @param v_secCelBody_N [m/s] secondary celestial body inertial velocity (ignored when not linked)
 @param r_BN_N [m] spacecraft inertial position
 @param v_BN_N [m/s] spacecraft inertial velocity
 @return attitude reference output
 */
CelestialTwoBodyPointOutput CelestialTwoBodyPointAlgorithm::update(const Eigen::Vector3d &r_celBody_N,
                                                                   const Eigen::Vector3d &v_celBody_N,
                                                                   const Eigen::Vector3d &r_secCelBody_N,
                                                                   const Eigen::Vector3d &v_secCelBody_N,
                                                                   const Eigen::Vector3d &r_BN_N,
                                                                   const Eigen::Vector3d &v_BN_N) const {
    const Eigen::Vector3d r_PB_N = r_celBody_N - r_BN_N;
    const Eigen::Vector3d v_PB_N = v_celBody_N - v_BN_N;

    Eigen::Vector3d r_SB_N{};
    Eigen::Vector3d v_SB_N{};

    float platAngDiff{}; /* Angle between r_PB_N and r_SB_N */
    if (this->cfg.getSecCelBodyIsLinked()) {
        r_SB_N = r_secCelBody_N - r_BN_N;
        v_SB_N = v_secCelBody_N - v_BN_N;

        const float dotProduct = static_cast<float>(r_SB_N.normalized().dot(r_PB_N.normalized()));
        platAngDiff = safeAcosf(dotProduct);
    }

    CelestialTwoBodyPointOutput attRefOut = this->rateAndAccelCalc(r_PB_N, v_PB_N, r_SB_N, v_SB_N);

    /*! - Cross the first bodies' states to get R_SB and v_SB if no secondary celestial body is included or
     if the two bodies are close to parallel or if the computed rate was higher than rate threshold */
    if (!this->cfg.getSecCelBodyIsLinked() || attRefOut.omega_RN_N.norm() > this->cfg.getRateThreshold() ||
        fabsf(platAngDiff) < this->cfg.getSingularityThreshold() ||
        fabsf(platAngDiff) > std::numbers::pi_v<float> - this->cfg.getSingularityThreshold()) {
        r_SB_N = r_PB_N.cross(v_PB_N);
        v_SB_N = Eigen::Vector3d::Zero();
        attRefOut = this->rateAndAccelCalc(r_PB_N, v_PB_N, r_SB_N, v_SB_N);
    }

    return attRefOut;
}

CelestialTwoBodyPointOutput CelestialTwoBodyPointAlgorithm::rateAndAccelCalc(const Eigen::Vector3d &r_PB_N,
                                                                             const Eigen::Vector3d &v_PB_N,
                                                                             const Eigen::Vector3d &r_SB_N,
                                                                             const Eigen::Vector3d &v_SB_N) {
    /* - Initial computations: R_n, v_n, a_n */
    const Eigen::Vector3d R_N = r_PB_N.cross(r_SB_N);
    const Eigen::Vector3d v_N = v_PB_N.cross(r_SB_N) + r_PB_N.cross(v_SB_N);
    const Eigen::Vector3d a_N = 2 * v_PB_N.cross(v_SB_N);

    /* - Reference Frame computation */
    const Eigen::Vector3f r1_N_hat = r_PB_N.normalized().cast<float>();
    const Eigen::Vector3f r3_N_hat = R_N.normalized().cast<float>();
    const Eigen::Vector3f r2_N_hat = (r3_N_hat.cross(r1_N_hat)).normalized();
    Eigen::Matrix3f dcm_RN{};
    dcm_RN.row(0) = r1_N_hat;
    dcm_RN.row(1) = r2_N_hat;
    dcm_RN.row(2) = r3_N_hat;

    CelestialTwoBodyPointOutput attRefOut{};
    attRefOut.sigma_RN = dcmToMrp(dcm_RN);

    /* - Reference base-vectors first time-derivative */
    const Eigen::Vector3f dr1_N_hat =
        (Eigen::Matrix3f::Identity() - r1_N_hat * r1_N_hat.transpose()) * (v_PB_N / r_PB_N.norm()).cast<float>();
    const Eigen::Vector3f dr3_N_hat =
        (Eigen::Matrix3f::Identity() - r3_N_hat * r3_N_hat.transpose()) * (v_N / R_N.norm()).cast<float>();
    const Eigen::Vector3f dr2_N_hat = dr3_N_hat.cross(r1_N_hat) + r3_N_hat.cross(dr1_N_hat);

    /* - Angular velocity computation */
    Eigen::Vector3f omega_RN_R{};
    omega_RN_R[0] = r3_N_hat.dot(dr2_N_hat);
    omega_RN_R[1] = r1_N_hat.dot(dr3_N_hat);
    omega_RN_R[2] = r2_N_hat.dot(dr1_N_hat);
    attRefOut.omega_RN_N = dcm_RN.transpose() * omega_RN_R;

    /* - Reference base-vectors second time-derivative */
    const Eigen::Vector3f ddr1_N_hat = -(2 * dr1_N_hat * r1_N_hat.transpose() + r1_N_hat * dr1_N_hat.transpose()) *
                                       (v_PB_N / r_PB_N.norm()).cast<float>();
    const Eigen::Vector3f ddr3_N_hat =
        ((Eigen::Matrix3f::Identity() - r3_N_hat * r3_N_hat.transpose()) * a_N.cast<float>() -
         (2 * dr3_N_hat * r3_N_hat.transpose() + r3_N_hat * dr3_N_hat.transpose()) * v_N.cast<float>()) /
        R_N.norm();
    const Eigen::Vector3f ddr2_N_hat =
        ddr3_N_hat.cross(r1_N_hat) + r3_N_hat.cross(ddr1_N_hat) + 2 * dr3_N_hat.cross(dr1_N_hat);

    /* - Angular acceleration computation */
    Eigen::Vector3f domega_RN_R{};
    domega_RN_R[0] = dr3_N_hat.dot(dr2_N_hat) + r3_N_hat.dot(ddr2_N_hat) - omega_RN_R.dot(dr1_N_hat);
    domega_RN_R[1] = dr1_N_hat.dot(dr3_N_hat) + r1_N_hat.dot(ddr3_N_hat) - omega_RN_R.dot(dr2_N_hat);
    domega_RN_R[2] = dr2_N_hat.dot(dr1_N_hat) + r2_N_hat.dot(ddr1_N_hat) - omega_RN_R.dot(dr3_N_hat);
    attRefOut.domega_RN_N = dcm_RN.transpose() * domega_RN_R;

    return attRefOut;
}
