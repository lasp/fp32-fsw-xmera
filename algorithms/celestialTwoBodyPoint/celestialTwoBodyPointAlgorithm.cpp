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

/*! This method resets the algorithm state. */
void CelestialTwoBodyPointAlgorithm::reInitialize() {}

/*! This method fully resets the algorithm state. */
void CelestialTwoBodyPointAlgorithm::reInitializeAll() { reInitialize(); }

/*! This method computes the attitude reference that points the primary axis at the primary
 celestial body while aligning a second axis as close as possible toward the secondary
 celestial body. It generates the commanded attitude and assumes that the control errors are
 computed downstream.
 @param primaryBodyState [m, m/s] primary celestial body inertial position and velocity
 @param secondaryBodyState [m, m/s] secondary celestial body inertial position and velocity
 @param spacecraftState [m, m/s] spacecraft inertial position and velocity
 @return attitude reference output
 */
// NOLINTBEGIN(bugprone-easily-swappable-parameters)
// bugprone-easily-swappable-parameters: the InertialStateInput inputs are documented in the header and
// follow the standard (primary, secondary, spacecraft) ordering.
CelestialTwoBodyPointOutput CelestialTwoBodyPointAlgorithm::update(const InertialStateInput &primaryBodyState,
                                                                   const InertialStateInput &secondaryBodyState,
                                                                   const InertialStateInput &spacecraftState) const {
    const Eigen::Vector3d r_PB_N = primaryBodyState.r_N - spacecraftState.r_N;
    const Eigen::Vector3d v_PB_N = primaryBodyState.v_N - spacecraftState.v_N;
    Eigen::Vector3d r_SB_N = secondaryBodyState.r_N - spacecraftState.r_N;
    Eigen::Vector3d v_SB_N = secondaryBodyState.v_N - spacecraftState.v_N;

    /*! Default reference output */
    CelestialTwoBodyPointOutput attRefOut{};

    /*! Return default reference output if either celestial body is not resolved (if r_PB_N or r_SB_N are zero) */
    const bool celestialBodiesResolved = r_PB_N.squaredNorm() >= kMinNormSq && r_SB_N.squaredNorm() >= kMinNormSq;

    if (celestialBodiesResolved) {
        /*! Compute angle between celestial bodies */
        const auto dotProduct1 = static_cast<float>(r_SB_N.normalized().dot(r_PB_N.normalized()));
        const float celestialBodySeparationAngle = safeAcosf(fabsf(dotProduct1)); /* Angle between r_PB_N and r_SB_N */

        bool constraintAxisValid = true;

        /*! Update r_SB_N and v_SB_N if celestial bodies are aligned, unless r_PB_N and v_PB_N are collinear (fallback
         * constraint axis undefined) */
        if (celestialBodySeparationAngle < this->cfg.getCelestialBodyAlignmentThreshold()) {
            const auto dotProduct2 = static_cast<float>(r_PB_N.normalized().dot(v_PB_N.normalized()));
            const float posVelSeparationAngle = safeAcosf(fabsf(dotProduct2)); /* Angle between r_PB_N and v_PB_N */
            constraintAxisValid = posVelSeparationAngle >= kSmallAngle;
            if (constraintAxisValid) {
                r_SB_N = r_PB_N.cross(v_PB_N);
                v_SB_N = Eigen::Vector3d::Zero();
            }
        }

        /*! Nominal algorithm flow if both celestial bodies are resolved and constraint axes are valid */
        if (constraintAxisValid) {
            attRefOut = rateAndAccelCalc(r_PB_N, v_PB_N, r_SB_N, v_SB_N);
        }
    }

    return attRefOut;
}
// NOLINTEND(bugprone-easily-swappable-parameters)

CelestialTwoBodyPointOutput CelestialTwoBodyPointAlgorithm::rateAndAccelCalc(const Eigen::Vector3d &r_PB_N,
                                                                             const Eigen::Vector3d &v_PB_N,
                                                                             const Eigen::Vector3d &r_SB_N,
                                                                             const Eigen::Vector3d &v_SB_N) {
    /* Compute normal vector to plane of r_PB_N and r_SB_N */
    const Eigen::Vector3d normalVec_N = r_PB_N.cross(r_SB_N);

    /* Compute inertial time derivative of normal vector */
    const Eigen::Vector3d normalVecDot_N = v_PB_N.cross(r_SB_N) + r_PB_N.cross(v_SB_N);

    /* Compute inertial acceleration of normal vector */
    const Eigen::Vector3d normalVecDDot_N = 2 * v_PB_N.cross(v_SB_N);

    /* Reference frame computation */
    const Eigen::Vector3d r1Hat_N = r_PB_N.normalized();
    const Eigen::Vector3d r3Hat_N = normalVec_N.normalized();
    const Eigen::Vector3d r2Hat_N = r3Hat_N.cross(r1Hat_N).normalized();
    Eigen::Matrix3d dcm_RN = Eigen::Matrix3d::Identity();
    dcm_RN.row(0) = r1Hat_N;
    dcm_RN.row(1) = r2Hat_N;
    dcm_RN.row(2) = r3Hat_N;

    // Construct algorithm output message
    CelestialTwoBodyPointOutput attRefOut{};
    attRefOut.sigma_RN = dcmToMrp(Eigen::Matrix3f(dcm_RN.cast<float>()));

    /* Compute inertial time derivative of reference frame basis vectors */
    const Eigen::Vector3d r1HatDot_N =
        (Eigen::Matrix3d::Identity() - r1Hat_N * r1Hat_N.transpose()) * v_PB_N / r_PB_N.norm();
    const Eigen::Vector3d r3HatDot_N =
        (Eigen::Matrix3d::Identity() - r3Hat_N * r3Hat_N.transpose()) * normalVecDot_N / normalVec_N.norm();
    const Eigen::Vector3d r2HatDot_N = r3HatDot_N.cross(r1Hat_N) + r3Hat_N.cross(r1HatDot_N);

    /* Reference angular velocity computation */
    Eigen::Vector3d omega_RN_R = Eigen::Vector3d::Zero();
    omega_RN_R[0] = r3Hat_N.dot(r2HatDot_N);
    omega_RN_R[1] = r1Hat_N.dot(r3HatDot_N);
    omega_RN_R[2] = r2Hat_N.dot(r1HatDot_N);
    attRefOut.omega_RN_N = (dcm_RN.transpose() * omega_RN_R).cast<float>();

    /* Compute inertial acceleration of reference frame basis vectors */
    const Eigen::Vector3d r1HatDDot_N =
        -(2 * r1HatDot_N * r1Hat_N.transpose() + r1Hat_N * r1HatDot_N.transpose()) * v_PB_N / r_PB_N.norm();
    const Eigen::Vector3d r3HatDDot_N =
        ((Eigen::Matrix3d::Identity() - r3Hat_N * r3Hat_N.transpose()) * normalVecDDot_N -
         (2 * r3HatDot_N * r3Hat_N.transpose() + r3Hat_N * r3HatDot_N.transpose()) * normalVecDot_N) /
        normalVec_N.norm();
    const Eigen::Vector3d r2HatDDot_N =
        r3HatDDot_N.cross(r1Hat_N) + r3Hat_N.cross(r1HatDDot_N) + 2 * r3HatDot_N.cross(r1HatDot_N);

    /* Reference angular acceleration computation */
    Eigen::Vector3d omegaDot_RN_R{};
    omegaDot_RN_R[0] = r3HatDot_N.dot(r2HatDot_N) + r3Hat_N.dot(r2HatDDot_N) - omega_RN_R.dot(r1HatDot_N);
    omegaDot_RN_R[1] = r1HatDot_N.dot(r3HatDot_N) + r1Hat_N.dot(r3HatDDot_N) - omega_RN_R.dot(r2HatDot_N);
    omegaDot_RN_R[2] = r2HatDot_N.dot(r1HatDot_N) + r2Hat_N.dot(r1HatDDot_N) - omega_RN_R.dot(r3HatDot_N);
    attRefOut.domega_RN_N = (dcm_RN.transpose() * omegaDot_RN_R).cast<float>();

    return attRefOut;
}
