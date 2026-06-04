#include "rwMotorTorqueAlgorithm.h"
#include "utilities/freestandingInvalidArgument.h"
#include <Eigen/LU>
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

/*! Builds the RW null-space projection [tau] = [I] - [Gs]^T([Gs][Gs]^T)^-1[Gs] from the available-wheel
 spin-axis matrix [Gs] (in-position, zero columns for unavailable wheels). Returns the zero matrix (despin
 disabled) when there is no usable null space: three or fewer available wheels, or wheels that do not span
 3-D so ([Gs][Gs]^T) would be singular. */
Eigen::Matrix<float, kMaxNumRw, kMaxNumRw> computeNullSpaceProjection(const Eigen::Matrix<float, 3, kMaxNumRw>& G_s_B,
                                                                      uint32_t numAvailRW) {
    if (numAvailRW <= 3U) {
        return Eigen::Matrix<float, kMaxNumRw, kMaxNumRw>::Zero();
    }

    const Eigen::Matrix3f GsGsT = G_s_B * G_s_B.transpose();
    const Eigen::JacobiSVD<Eigen::Matrix3f> gsSvd(GsGsT);
    const Eigen::Vector3f gsSingularValues = gsSvd.singularValues();
    constexpr float kSpanRelativeTol = 1e-6F;
    if (gsSingularValues(2) <= gsSingularValues(0) * kSpanRelativeTol) {
        return Eigen::Matrix<float, kMaxNumRw, kMaxNumRw>::Zero();
    }

    return Eigen::Matrix<float, kMaxNumRw, kMaxNumRw>{Eigen::Matrix<float, kMaxNumRw, kMaxNumRw>::Identity() -
                                                      G_s_B.transpose() * GsGsT.inverse() * G_s_B};
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

    // Controllability cross-check: every control axis must be reachable by the available reaction wheels.
    // A control axis with a significant projection onto the left-null-space of [CGs] is not reachable.
    const Eigen::JacobiSVD<Eigen::MatrixXf> svd(CGs.topRows(numControlAxes), Eigen::ComputeFullU);
    const Eigen::VectorXf& singularValues = svd.singularValues();
    const Eigen::MatrixXf& leftSingularVectors = svd.matrixU();
    const float singularValueTol = singularValues(0) * std::numeric_limits<float>::epsilon() *
                                   static_cast<float>(std::max(numControlAxes, kMaxNumRw));
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

    // Fold the control-axis projection and minimum-norm pseudo-inverse into one map. Unavailable wheels are
    // zero columns of [CGs], so their map rows come out zero, and the inverted Gram [CGs][CGs].T is the small
    // numControlAxes x numControlAxes block (full rank by the check above).
    const Eigen::MatrixXf CGsActive = CGs.topRows(numControlAxes);
    RwMotorTorqueMapping mapping{};
    mapping.motorTorqueMap = CGsActive.transpose() * (CGsActive * CGsActive.transpose()).inverse() *
                             (-compactControlAxes.topRows(numControlAxes));

    mapping.tau = computeNullSpaceProjection(G_s_B, numAvailRW);

    // Zero the despin rows of excluded wheels: their [Gs] columns are zero, so [tau] would otherwise leave
    // identity rows that inject the raw despin command onto an excluded wheel.
    for (uint32_t i = 0U; i < kMaxNumRw; ++i) {
        if (i >= rwConfiguration.numRW || wheelsAvailability[i] != AVAILABLE) {
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

RwMotorTorqueAlgorithm::RwMotorTorqueAlgorithm(const RwMotorTorqueConfig& config) : cfg(config) {
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
