#include "rwMotorTorqueAlgorithm.h"
#include "utilities/freestandingInvalidArgument.h"
#include <Eigen/SVD>
#include <algorithm>
#include <cstdint>
#include <limits>
#include <optional>

namespace {

// The configuration-dependent maps precomputed by computeRwMapping() and applied by update().
struct RwMotorTorqueMapping {
    Eigen::Matrix<float, kMaxNumRw, 3> motorTorqueMap{
        Eigen::Matrix<float, kMaxNumRw, 3>::Zero()};  //!< [-] body control torque -> per-RW motor torques
    Eigen::Matrix<float, kMaxNumRw, kMaxNumRw> tau{
        Eigen::Matrix<float, kMaxNumRw, kMaxNumRw>::Zero()};  //!< [-] RW null-space projection for despin
};

// Configurations whose pseudo-inverted operator -- [CGs] for the control map, [Gs] for the null-space
// projector -- has a condition number above 1 / kConditioningTol = 100 are rejected, so a near-degenerate
// layout (which would amplify command and fp32 error, or yield an unreliable despin) can never be configured.
constexpr float kConditioningTol = 1e-2F;

/*! Builds the RW null-space projection [tau], the orthogonal projector onto the null space of the available-
 wheel spin-axis matrix [Gs] (in-position, zero columns for unavailable wheels). [tau] is built from the right
 singular vectors of [Gs] -- [tau] = [I] - [Vr][Vr]^T where [Vr] spans the row space -- so [Gs][tau] == 0 to
 machine precision. Returns the zero matrix (despin disabled) when there are three or fewer available wheels,
 and nullopt when more than three available wheels are ill-conditioned (cond([Gs]) > 100, which also covers a
 rank-deficient array that does not span 3-D). */
std::optional<Eigen::Matrix<float, kMaxNumRw, kMaxNumRw>> computeNullSpaceProjection(
    const Eigen::Matrix<float, 3, kMaxNumRw>& G_s_B,
    uint32_t numAvailRW) {
    if (numAvailRW <= 3U) {
        return Eigen::Matrix<float, kMaxNumRw, kMaxNumRw>::Zero();
    }

    const Eigen::JacobiSVD<Eigen::Matrix<float, 3, kMaxNumRw>> gsSvd(G_s_B, Eigen::ComputeFullV);
    const Eigen::Vector3f& gsSingularValues = gsSvd.singularValues();
    if (gsSingularValues(2) <= gsSingularValues(0) * kConditioningTol) {
        return std::nullopt;
    }

    const Eigen::Matrix<float, kMaxNumRw, 3> Vr = gsSvd.matrixV().leftCols<3>();
    return Eigen::Matrix<float, kMaxNumRw, kMaxNumRw>{Eigen::Matrix<float, kMaxNumRw, kMaxNumRw>::Identity() -
                                                      Vr * Vr.transpose()};
}

/*! Computes the per-RW motor-torque map and the null-space projection [tau] from a validated, canonicalized
 configuration (orthonormal control axes, unit spin axes). Returns nullopt when a control axis is not
 reachable by the available reaction wheels (rank-deficient control mapping). */
std::optional<RwMotorTorqueMapping> computeRwMapping(const Eigen::Matrix3f& controlAxes_B,
                                                     const RwMotorTorqueArrayConfiguration& rwConfiguration,
                                                     const RwMotorTorqueAvailability& availability) {
    const std::array<FSWdeviceAvailability, kMaxNumRw>& wheelsAvailability = availability.wheelAvailability;

    // Compact the non-zero (controlled) rows to the top; only the controlled subspace matters, so the row
    // positions in the configured matrix are irrelevant.
    Eigen::Matrix3f compactControlAxes{Eigen::Matrix3f::Zero()};
    uint32_t numControlAxes = 0U;
    for (uint32_t i = 0U; i < 3U; ++i) {
        if (controlAxes_B.row(i).norm() > 0.0F) {
            compactControlAxes.row(numControlAxes) = controlAxes_B.row(i);
            numControlAxes += 1U;
        }
    }

    // [Gs] from the available RWs, each in its original column (unavailable wheels stay zero). The config
    // stores unit spin axes, so no normalization is needed here.
    Eigen::Matrix<float, 3, kMaxNumRw> G_s_B{Eigen::Matrix<float, 3, kMaxNumRw>::Zero()};
    uint32_t numAvailRW = 0U;
    for (uint32_t i = 0U; i < rwConfiguration.numRW; ++i) {
        if (wheelsAvailability[i] == AVAILABLE) {
            G_s_B.col(i) = rwConfiguration.GsMatrix_B.col(i);
            numAvailRW += 1U;
        }
    }

    const Eigen::Matrix<float, 3, kMaxNumRw> CGs = compactControlAxes * G_s_B;
    const Eigen::MatrixXf CGsActive = CGs.topRows(numControlAxes);

    // SVD of the control mapping [CGs] = [CB][Gs] (numControlAxes x kMaxNumRw, with zero columns for
    // unavailable wheels), reused for the controllability check and the pseudo-inverse below. Singular values
    // below a relative tolerance are treated as zero.
    const Eigen::JacobiSVD<Eigen::MatrixXf> svd(CGsActive, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const Eigen::VectorXf& singularValues = svd.singularValues();
    const Eigen::MatrixXf& leftSingularVectors = svd.matrixU();
    const float singularValueTol = singularValues(0) * std::numeric_limits<float>::epsilon() *
                                   static_cast<float>(std::max(numControlAxes, kMaxNumRw));

    // Controllability cross-check: every control axis must be reachable by the available reaction wheels.
    // A control axis with a significant projection onto the left-null-space of [CGs] is not reachable.
    constexpr float kControllabilityResidualSqTol = 1e-6F;
    for (uint32_t axis = 0U; axis < numControlAxes; ++axis) {
        float nullSpaceResidualSq = 0.0F;
        for (uint32_t k = 0U; k < numControlAxes; ++k) {
            if (singularValues(k) <= singularValueTol) {
                nullSpaceResidualSq += leftSingularVectors(axis, k) * leftSingularVectors(axis, k);
            }
        }
        if (nullSpaceResidualSq > kControllabilityResidualSqTol) {
            return std::nullopt;
        }
    }

    // Reject ill-conditioned control geometry: even at full rank, a small smallest-to-largest singular value
    // ratio makes the pseudo-inverse amplify both the command and fp32 error. cond([CGs]) = sv(0) / sv(min).
    if (singularValues(numControlAxes - 1U) <= singularValues(0) * kConditioningTol) {
        return std::nullopt;
    }

    // Control map = pseudo-inverse([CGs]) * (-[CB]) via a truncated SVD ([V][S^-1][U]^T), folding the control-
    // axis projection and the minimum-norm inverse into one matrix.
    Eigen::VectorXf invSingularValues = Eigen::VectorXf::Zero(singularValues.size());
    for (Eigen::Index i = 0; i < singularValues.size(); ++i) {
        if (singularValues(i) > singularValueTol) {
            invSingularValues(i) = 1.0F / singularValues(i);
        }
    }
    RwMotorTorqueMapping mapping{};
    mapping.motorTorqueMap = svd.matrixV() * invSingularValues.asDiagonal() * leftSingularVectors.transpose() *
                             (-compactControlAxes.topRows(numControlAxes));

    const std::optional<Eigen::Matrix<float, kMaxNumRw, kMaxNumRw>> tau = computeNullSpaceProjection(G_s_B, numAvailRW);
    if (!tau.has_value()) {
        return std::nullopt;
    }
    mapping.tau = *tau;

    // Zero the rows of excluded wheels (beyond numRW or unavailable). Their [CGs]/[Gs] columns are zero, so
    // these rows are zero in exact arithmetic; mask them so an excluded wheel is commanded exactly zero torque.
    for (uint32_t i = 0U; i < kMaxNumRw; ++i) {
        if (i >= rwConfiguration.numRW || wheelsAvailability[i] != AVAILABLE) {
            mapping.motorTorqueMap.row(i).setZero();
            mapping.tau.row(i).setZero();
        }
    }

    return mapping;
}

}  // namespace

bool RwMotorTorqueConfig::isValidMapping(const Eigen::Matrix3f& controlAxes_B,
                                         const RwMotorTorqueArrayConfiguration& rwConfiguration,
                                         const RwMotorTorqueAvailability& availability) {
    return computeRwMapping(controlAxes_B, rwConfiguration, availability).has_value();
}

RwMotorTorqueAlgorithm::RwMotorTorqueAlgorithm(const RwMotorTorqueConfig& config)  // NOLINT(modernize-pass-by-value)
    : cfg(config) {
    const std::optional<RwMotorTorqueMapping> mapping =
        computeRwMapping(this->cfg.getControlAxes(), this->cfg.getRwConfiguration(), this->cfg.getAvailability());
    if (mapping.has_value()) {
        this->motorTorqueMap = mapping->motorTorqueMap;
        this->tau = mapping->tau;
    }
}

void RwMotorTorqueAlgorithm::setConfig(const RwMotorTorqueConfig& config) {
    this->cfg = config;
    const std::optional<RwMotorTorqueMapping> mapping =
        computeRwMapping(this->cfg.getControlAxes(), this->cfg.getRwConfiguration(), this->cfg.getAvailability());
    if (mapping.has_value()) {
        this->motorTorqueMap = mapping->motorTorqueMap;
        this->tau = mapping->tau;
    }
}

/*! Maps the commanded body torque to per-RW motor torques and adds the null-space despin torque.
 @param Lr_B commanded control torque on the spacecraft, body frame [N-m]
 @param speeds current and desired RW speeds for the despin term
 @return per-RW motor torques [N-m]
 */
Eigen::Vector<float, kMaxNumRw> RwMotorTorqueAlgorithm::update(const Eigen::Vector3f& Lr_B,
                                                               const RwMotorTorqueSpeeds& speeds) const {
    const Eigen::Vector<float, kMaxNumRw> controlTorque = this->motorTorqueMap * Lr_B;

    // Null-space despin: [tau] projects the wheel-speed feedback so it adds no body torque.
    const Eigen::Vector<float, kMaxNumRw> d = -this->cfg.getOmegaGain() * (speeds.rwSpeeds - speeds.rwDesiredSpeeds);
    const Eigen::Vector<float, kMaxNumRw> nullSpaceTorque = this->tau * d;

    return controlTorque + nullSpaceTorque;
}
