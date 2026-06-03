#include "celestialTwoBodyPointAlgorithm.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/rigidBodyKinematics.hpp"
#include "utilities/fsw/safeMath.h"

void CelestialTwoBodyPointAlgorithm::reset(const bool secCelBodyIsLinkedIn) {
    this->secCelBodyIsLinked = secCelBodyIsLinkedIn;
}

/*! This method computes the attitude reference that points the primary axis at the primary
 celestial body while aligning a second axis as close as possible toward the secondary
 celestial body. It generates the commanded attitude and assumes that the control errors are
 computed downstream.
 @param celBodyIn primary celestial body ephemeris
 @param secCelBodyIn secondary celestial body ephemeris (ignored when not linked)
 @param transNavIn spacecraft translational navigation solution
 @return attitude reference message payload
 */
AttRefMsgF32Payload CelestialTwoBodyPointAlgorithm::update(const EphemerisMsgF32Payload &celBodyIn,
                                                           const EphemerisMsgF32Payload &secCelBodyIn,
                                                           const NavTransMsgF32Payload &transNavIn) const {
    const Eigen::Vector3d r_PB_N =
        cArrayToEigenVector3(celBodyIn.r_BdyZero_N) - cArrayToEigenVector3(transNavIn.r_BN_N);
    const Eigen::Vector3d v_PB_N =
        cArrayToEigenVector3(celBodyIn.v_BdyZero_N) - cArrayToEigenVector3(transNavIn.v_BN_N);

    Eigen::Vector3d r_SB_N{};
    Eigen::Vector3d v_SB_N{};

    float platAngDiff{}; /* Angle between r_PB_N and r_SB_N */
    if (this->secCelBodyIsLinked) {
        r_SB_N = cArrayToEigenVector3(secCelBodyIn.r_BdyZero_N) - cArrayToEigenVector3(transNavIn.r_BN_N);
        v_SB_N = cArrayToEigenVector3(secCelBodyIn.v_BdyZero_N) - cArrayToEigenVector3(transNavIn.v_BN_N);

        const float dotProduct = r_SB_N.normalized().dot(r_PB_N.normalized());
        platAngDiff = safeAcosf(dotProduct);
    }

    AttRefMsgF32Payload attRefOut = this->rateAndAccelCalc(r_PB_N, v_PB_N, r_SB_N, v_SB_N);

    /*! - Cross the first bodies' states to get R_SB and v_SB if no secondary celestial body is included or
     if the two bodies are close to parallel or if the computed rate was higher than rate threshold */
    if (!this->secCelBodyIsLinked || cArrayToEigenVector3(attRefOut.omega_RN_N).norm() > this->rateThreshold ||
        fabs(platAngDiff) < this->singularityThreshold || fabs(platAngDiff) > M_PI - this->singularityThreshold) {
        r_SB_N = r_PB_N.cross(v_PB_N);
        v_SB_N = Eigen::Vector3d::Zero();
        attRefOut = this->rateAndAccelCalc(r_PB_N, v_PB_N, r_SB_N, v_SB_N);
    }

    return attRefOut;
}

AttRefMsgF32Payload CelestialTwoBodyPointAlgorithm::rateAndAccelCalc(const Eigen::Vector3d &r_PB_N,
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
    const Eigen::Vector3f r2_N_hat = (r3_N_hat.cross(r1_N_hat)).normalized().cast<float>();
    Eigen::Matrix3f dcm_RN{};
    dcm_RN.row(0) = r1_N_hat;
    dcm_RN.row(1) = r2_N_hat;
    dcm_RN.row(2) = r3_N_hat;

    AttRefMsgF32Payload attRefOut{};
    const Eigen::Vector3f sigma_RN = dcmToMrp(dcm_RN);
    eigenVectorToCArray(sigma_RN, attRefOut.sigma_RN);

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
    const Eigen::Vector3f omega_RN_N = dcm_RN.transpose() * omega_RN_R;
    eigenVectorToCArray(omega_RN_N, attRefOut.omega_RN_N);

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
    const Eigen::Vector3f domega_RN_N = dcm_RN.transpose() * domega_RN_R;
    eigenVectorToCArray(domega_RN_N, attRefOut.domega_RN_N);

    return attRefOut;
}

/**
 * @brief Set the singularity threshold
 * @param singularityThresholdIn [rad] angle threshold below which the constraint axis is fixed
 */
void CelestialTwoBodyPointAlgorithm::setSingularityThreshold(const float singularityThresholdIn) {
    if (singularityThresholdIn < 0.0) {
        FSW_THROW_INVALID_ARGUMENT("Singularity threshold must not be negative");
    }
    this->singularityThreshold = singularityThresholdIn;
}

/**
 * @brief Get the singularity threshold
 * @return [rad] angle threshold below which the constraint axis is fixed
 */
float CelestialTwoBodyPointAlgorithm::getSingularityThreshold() const { return this->singularityThreshold; }

/**
 * @brief Set the rate threshold
 * @param rateThresholdIn [rad/s] rate threshold above which the constraint axis is fixed
 */
void CelestialTwoBodyPointAlgorithm::setRateThreshold(const float rateThresholdIn) {
    if (rateThresholdIn < 0.0) {
        FSW_THROW_INVALID_ARGUMENT("Rate threshold must not be negative");
    }
    this->rateThreshold = rateThresholdIn;
}

/**
 * @brief Get the rate threshold
 * @return [rad/s] rate threshold above which the constraint axis is fixed
 */
float CelestialTwoBodyPointAlgorithm::getRateThreshold() const { return this->rateThreshold; }
