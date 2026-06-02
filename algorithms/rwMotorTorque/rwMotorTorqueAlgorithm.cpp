#include "rwMotorTorqueAlgorithm.h"
#include "utilities/freestandingInvalidArgument.h"
#include <Eigen/LU>
#include <cstdint>

RwMotorTorqueAlgorithm::RwMotorTorqueAlgorithm(const RwMotorTorqueConfig& config) : cfg(config) {}

void RwMotorTorqueAlgorithm::setConfig(const RwMotorTorqueConfig& config) { this->cfg = config; }

/*! Computes the reaction wheel torques given a commanded torque on the spacecraft
 @return Eigen::Vector<float, kMaxNumRw> commanded RW motor torques [N-m]
 @param Lr_B total commanded control torque on the spacecraft in body-frame components
 */
Eigen::Vector<float, kMaxNumRw> RwMotorTorqueAlgorithm::update(const Eigen::Vector3f& Lr_B) const {
    return this->motorTorqueMap * Lr_B;
}

/*! Precomputes the constant map from the commanded body torque to the per-RW motor torques from the
 reaction-wheel configuration and availability. Must be called before update().
 @return void
 @param rwConfiguration reaction-wheel spin-axis configuration in body-frame components
 @param availability per-wheel reaction-wheel availability (a default-constructed value marks every wheel AVAILABLE)
 */
void RwMotorTorqueAlgorithm::computeRwMapping(const RwMotorTorqueArrayConfiguration& rwConfiguration,
                                              const RwMotorTorqueAvailability& availability) {
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
    const std::array<FSWdeviceAvailability, kMaxNumRw>& wheelsAvailability = availability.wheelAvailability;
    Eigen::Matrix<float, 3, kMaxNumRw> G_s_B{Eigen::Matrix<float, 3, kMaxNumRw>::Zero()};
    uint32_t numAvailRW = 0U;
    for (uint32_t i = 0U; i < rwConfiguration.numRW; ++i) {
        if (wheelsAvailability[i] == AVAILABLE) {
            G_s_B.col(numAvailRW) = rwConfiguration.GsMatrix_B.col(i).normalized();
            numAvailRW += 1U;
        }
    }

    const Eigen::Matrix<float, 3, kMaxNumRw> CGs = controlAxes_B * G_s_B;
    const Eigen::FullPivLU<Eigen::MatrixXf> lu_decomp(CGs);
    if (static_cast<uint32_t>(lu_decomp.rank()) < numControlAxes) {
        FSW_THROW_INVALID_ARGUMENT("rwMotorTorque: control mapping matrix [CB][G_s] is not full rank.");
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
