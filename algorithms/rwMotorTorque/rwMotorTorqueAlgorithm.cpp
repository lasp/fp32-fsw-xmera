#include "rwMotorTorqueAlgorithm.h"
#include "utilities/freestandingInvalidArgument.h"
#include <Eigen/LU>
#include <cstdint>

RwMotorTorqueAlgorithm::RwMotorTorqueAlgorithm(const RwMotorTorqueConfig& config) : cfg(config) {}

void RwMotorTorqueAlgorithm::setConfig(const RwMotorTorqueConfig& config) { this->cfg = config; }

/*! This method configures the module by populating any necessary class members.
 @return void
 @param rwConfig reaction-wheel spin-axis configuration in body-frame components
 @param availability per-wheel reaction-wheel availability
 @param rwAvailIsLinked boolean indicating whether RW availability information is provided
 */
void RwMotorTorqueAlgorithm::configure(const RwMotorTorqueArrayConfig& rwConfig,
                                       const RwMotorTorqueAvailability& availability,
                                       bool rwAvailIsLinked) {
    /*!- count the number of controlled axes. The control axes mapping matrix is already validated by
     RwMotorTorqueConfig (finite, filled top to bottom, at least one axis), so a simple count suffices. */
    const Eigen::Matrix3f& controlAxes_B = this->cfg.getControlAxes();
    this->numControlAxes = 0U;
    for (uint32_t i = 0U; i < 3U; ++i) {
        if (controlAxes_B.row(i).norm() > 0.0F) {
            this->numControlAxes += 1U;
        }
    }

    /*! - Store static RW config data in module variables */
    this->numRW = rwConfig.numRW;

    /*! - Build the [Gs] projection matrix with the available RWs.
     If no info is provided about RW availability we assume that all are available. */
    Eigen::Matrix<float, 3, kMaxNumRw> G_s_B{Eigen::Matrix<float, 3, kMaxNumRw>::Zero()};
    if (rwAvailIsLinked) {
        this->wheelsAvailability = availability.wheelAvailability;
        uint32_t numAvailWheels = 0U;
        for (uint32_t i = 0U; i < this->numRW; ++i) {
            if (this->wheelsAvailability[i] == AVAILABLE) {
                G_s_B.col(numAvailWheels) = rwConfig.GsMatrix_B.col(i).normalized();
                numAvailWheels += 1U;
            }
        }
        /*! - update the number of currently available RWs */
        this->numAvailRW = numAvailWheels;
    } else {
        this->wheelsAvailability.fill(AVAILABLE);
        for (uint32_t i = 0U; i < this->numRW; ++i) {
            G_s_B.col(i) = rwConfig.GsMatrix_B.col(i).normalized();
        }
        this->numAvailRW = this->numRW;
    }

    this->CGs = controlAxes_B * G_s_B;
    const Eigen::FullPivLU<Eigen::MatrixXf> lu_decomp(this->CGs);
    const auto controlMappingRank = static_cast<uint32_t>(lu_decomp.rank());

    if (controlMappingRank < this->numControlAxes) {
        FSW_THROW_INVALID_ARGUMENT("rwMotorTorque: control mapping matrix [CB][G_s] is not full rank.");
    }
}

/*! Computes the reaction wheel torques given a commanded torque on the spacecraft
 @return Eigen::Vector<float, kMaxNumRw> commanded RW motor torques [N-m]
 @param Lr_B total commanded control torque on the spacecraft in body-frame components
 */
Eigen::Vector<float, kMaxNumRw> RwMotorTorqueAlgorithm::update(const Eigen::Vector3f& Lr_B) const {
    /*! - zero RW motor torque variable */
    Eigen::Vector<float, kMaxNumRw> us = Eigen::Vector<float, kMaxNumRw>::Zero();

    /*! - Compute minimum norm inverse for us = [CGs].T inv([CGs][CGs].T) [Lr_C] */
    const uint32_t numRows = this->numControlAxes;
    const uint32_t numCols = this->numAvailRW;

    Eigen::Vector3f Lr_C{Eigen::Vector3f::Zero()};
    Lr_C.head(numRows) = -this->cfg.getControlAxes().topRows(numRows) * Lr_B;

    Eigen::Vector<float, kMaxNumRw> us_avail{Eigen::Vector<float, kMaxNumRw>::Zero()};
    us_avail.topRows(numCols) =
        this->CGs.topLeftCorner(numRows, numCols).transpose() *
        (this->CGs.topLeftCorner(numRows, numCols) * this->CGs.topLeftCorner(numRows, numCols).transpose()).inverse() *
        Lr_C.topRows(numRows);

    /*! - map the desired RW motor torques to the available RWs */
    uint32_t j = 0U;
    for (uint32_t i = 0U; i < this->numRW; ++i) {
        if (this->wheelsAvailability[i] == AVAILABLE) {
            us[i] = us_avail[j];
            j += 1U;
        }
    }

    return us;
}
