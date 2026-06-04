#include "rwMotorTorqueAlgorithm.h"
#include "utilities/freestandingInvalidArgument.h"
#include <Eigen/LU>
#include <Eigen/SVD>
#include <algorithm>
#include <cstdint>
#include <limits>

RwMotorTorqueAlgorithm::RwMotorTorqueAlgorithm(const RwMotorTorqueConfig& config) : cfg(config) {
    this->computeRwMapping();
}

void RwMotorTorqueAlgorithm::setConfig(const RwMotorTorqueConfig& config) {
    this->cfg = config;
    this->computeRwMapping();
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

/*! Precomputes the constant map from the commanded body torque to the per-RW motor torques from the
 configuration (control axes, reaction-wheel spin axes, and availability). Called from the constructor
 and from setConfig(). Throws if the resulting control mapping matrix is not full rank.
 @return void
 */
void RwMotorTorqueAlgorithm::computeRwMapping() {
    const RwMotorTorqueArrayConfiguration& rwConfiguration = this->cfg.getRwConfiguration();
    const std::array<FSWdeviceAvailability, kMaxNumRw>& wheelsAvailability =
        this->cfg.getAvailability().wheelAvailability;

    /*!- Gather the control axes. RwMotorTorqueConfig has already validated them (finite, orthonormal,
     at least one axis); the non-zero rows may sit in any position, so compact them to the top here.
     Only the controlled subspace matters, so the row positions in the configured matrix are irrelevant. */
    const Eigen::Matrix3f& configuredControlAxes_B = this->cfg.getControlAxes();
    Eigen::Matrix3f controlAxes_B{Eigen::Matrix3f::Zero()};
    uint32_t numControlAxes = 0U;
    for (uint32_t i = 0U; i < 3U; ++i) {
        if (configuredControlAxes_B.row(i).norm() > 0.0F) {
            controlAxes_B.row(numControlAxes) = configuredControlAxes_B.row(i);
            numControlAxes += 1U;
        }
    }

    // [Gs] from the available RWs, each in its original column (unavailable wheels stay zero).
    Eigen::Matrix<float, 3, kMaxNumRw> G_s_B{Eigen::Matrix<float, 3, kMaxNumRw>::Zero()};
    uint32_t numAvailRW = 0U;
    for (uint32_t i = 0U; i < rwConfiguration.numRW; ++i) {
        if (wheelsAvailability[i] == AVAILABLE) {
            G_s_B.col(i) = rwConfiguration.GsMatrix_B.col(i).normalized();
            numAvailRW += 1U;
        }
    }

    const Eigen::Matrix<float, 3, kMaxNumRw> CGs = controlAxes_B * G_s_B;

    /*! - Controllability cross-check: every control axis must be reachable by the available reaction
     wheels. Decompose the control mapping [CGs] = [CB][Gs] (numControlAxes x kMaxNumRw, with zero columns
     for unavailable wheels) with an SVD. Singular values below a relative tolerance are treated as
     zero; a control axis with a significant projection onto the corresponding left singular vectors
     (the left-null-space) cannot be produced by the available wheels and is rejected. */
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
            FSW_THROW_INVALID_ARGUMENT(
                "rwMotorTorque: a control axis is not controllable by the available reaction wheels "
                "(control mapping matrix [CB][G_s] is rank deficient).");
        }
    }

    /*! - Fold the control-axis projection and minimum-norm pseudo-inverse into one map. Unavailable wheels
     are zero columns of [CGs], so their map rows come out zero with no scatter step, and the inverted Gram
     [CGs][CGs].T is the small numControlAxes x numControlAxes block (full rank by the check above). */
    const Eigen::MatrixXf CGsActive = CGs.topRows(numControlAxes);
    this->motorTorqueMap = CGsActive.transpose() * (CGsActive * CGsActive.transpose()).inverse() *
                           (-controlAxes_B.topRows(numControlAxes));

    this->computeNullSpaceProjection(G_s_B, numAvailRW);

    // Zero the despin rows of excluded wheels. computeNullSpaceProjection works from [Gs] alone and can't
    // tell an unavailable wheel from a degenerate zero-axis available wheel, so mask here.
    for (uint32_t i = 0U; i < kMaxNumRw; ++i) {
        if (i >= rwConfiguration.numRW || wheelsAvailability[i] != AVAILABLE) {
            this->tau.row(i).setZero();
        }
    }
}

/*! Precomputes the RW null-space projection [tau] = [I] - [Gs]^T([Gs][Gs]^T)^-1[Gs] from the shared
 available-wheel [Gs] (in-position, zero columns for unavailable wheels). Wheels with zero columns come out
 as identity rows; the caller masks the excluded ones. Left zero unless more than three wheels are available
 and span 3-D, so ([Gs][Gs]^T) is never inverted while singular.
 @param G_s_B available-wheel spin-axis matrix shared with the control mapping
 @param numAvailRW number of available reaction wheels
 */
void RwMotorTorqueAlgorithm::computeNullSpaceProjection(const Eigen::Matrix<float, 3, kMaxNumRw>& G_s_B,
                                                        uint32_t numAvailRW) {
    this->tau.setZero();

    // No null space unless more than three wheels are available.
    if (numAvailRW <= 3U) {
        return;
    }

    // ([Gs][Gs]^T) is invertible only when the available wheels span 3-D; a rank-deficient array leaves [tau] zero.
    const Eigen::Matrix3f GsGsT = G_s_B * G_s_B.transpose();
    const Eigen::JacobiSVD<Eigen::Matrix3f> gsSvd(GsGsT);
    const Eigen::Vector3f gsSingularValues = gsSvd.singularValues();
    constexpr float kSpanRelativeTol = 1e-6F;
    if (gsSingularValues(2) <= gsSingularValues(0) * kSpanRelativeTol) {
        return;
    }

    this->tau = Eigen::Matrix<float, kMaxNumRw, kMaxNumRw>::Identity() - G_s_B.transpose() * GsGsT.inverse() * G_s_B;
}
