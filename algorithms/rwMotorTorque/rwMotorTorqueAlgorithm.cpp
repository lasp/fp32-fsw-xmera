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

/*! Computes the reaction wheel torques given a commanded torque on the spacecraft
 @return Eigen::Vector<float, kMaxNumRw> commanded RW motor torques [N-m]
 @param Lr_B total commanded control torque on the spacecraft in body-frame components
 */
Eigen::Vector<float, kMaxNumRw> RwMotorTorqueAlgorithm::update(const Eigen::Vector3f& Lr_B) const {
    return this->motorTorqueMap * Lr_B;
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

    /*!- count the number of controlled axes. The control axes mapping matrix is already validated by
     RwMotorTorqueConfig (finite, filled top to bottom, at least one axis), so a simple count suffices. */
    const Eigen::Matrix3f& controlAxes_B = this->cfg.getControlAxes();
    uint32_t numControlAxes = 0U;
    for (uint32_t i = 0U; i < 3U; ++i) {
        if (controlAxes_B.row(i).norm() > 0.0F) {
            numControlAxes += 1U;
        }
    }

    /*! - Build the [Gs] projection matrix from the available RWs. A wheel left at its default AVAILABLE
     state (i.e. no availability message was provided) is always included. */
    Eigen::Matrix<float, 3, kMaxNumRw> G_s_B{Eigen::Matrix<float, 3, kMaxNumRw>::Zero()};
    uint32_t numAvailRW = 0U;
    for (uint32_t i = 0U; i < rwConfiguration.numRW; ++i) {
        if (wheelsAvailability[i] == AVAILABLE) {
            G_s_B.col(numAvailRW) = rwConfiguration.GsMatrix_B.col(i).normalized();
            numAvailRW += 1U;
        }
    }

    const Eigen::Matrix<float, 3, kMaxNumRw> CGs = controlAxes_B * G_s_B;

    /*! - Controllability cross-check: every control axis must be reachable by the available reaction
     wheels. Decompose the control mapping [CGs] = [CB][Gs] (numControlAxes x kMaxNumRw, trailing columns
     zero for unavailable wheels) with an SVD. Singular values below a relative tolerance are treated as
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

    /*! - Precompute the constant map from the commanded body torque to the available RW motor torques:
     us_avail = [CGs].T inv([CGs][CGs].T) (-[CB] Lr_B). The control-axis projection and minimum-norm
     pseudo-inverse only depend on the configuration, so they are folded into a single matrix here. A
     full-rank [CGs] guarantees numAvailRW >= numControlAxes >= 1, so the active block is non-empty. */
    const Eigen::MatrixXf CGsAvail = CGs.topLeftCorner(numControlAxes, numAvailRW);
    const Eigen::MatrixXf availableMotorTorqueMap =
        CGsAvail.transpose() * (CGsAvail * CGsAvail.transpose()).inverse() * (-controlAxes_B.topRows(numControlAxes));

    /*! - Scatter the available-wheel rows back onto the full RW array; rows of unavailable wheels stay zero. */
    this->motorTorqueMap.setZero();
    uint32_t j = 0U;
    for (uint32_t i = 0U; i < rwConfiguration.numRW; ++i) {
        if (wheelsAvailability[i] == AVAILABLE) {
            this->motorTorqueMap.row(i) = availableMotorTorqueMap.row(j);
            j += 1U;
        }
    }
}
