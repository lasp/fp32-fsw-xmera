#ifndef F32XMERA_RW_MOTOR_TORQUE_ALGORITHM_H
#define F32XMERA_RW_MOTOR_TORQUE_ALGORITHM_H

#include "msgPayloadDef/definitions.h"
#include "utilities/freestandingInvalidArgument.h"
#include <fswAlgorithms/fswUtilities/fswDefinitions.h>

#include <Eigen/Core>
#include <array>
#include <cstdint>

inline constexpr uint32_t kMaxNumRw = RW_EFF_CNT;

/*! @brief Reaction-wheel spin-axis configuration in body-frame components. */
struct RwMotorTorqueArrayConfig {
    uint32_t numRW{};  //!< [-] number of reaction wheels on the vehicle
    Eigen::Matrix<float, 3, kMaxNumRw> GsMatrix_B{
        Eigen::Matrix<float, 3, kMaxNumRw>::Zero()};  //!< [-] RW spin axes in body frame, one column per wheel
};

/*! @brief Per-wheel reaction-wheel availability. */
struct RwMotorTorqueAvailability {
    std::array<FSWdeviceAvailability, kMaxNumRw>
        wheelAvailability{};  //!< [-] AVAILABLE / UNAVAILABLE state of each reaction wheel
};

/*!
 * @brief Validated configuration for the RW motor torque algorithm.
 *
 * An instance of this class can only exist if the control axes mapping matrix is finite, defines at
 * least one control axis, and is filled from top to bottom (any zero, uncontrolled axes at the
 * bottom). Construct via RwMotorTorqueConfig::create(...).
 */
class RwMotorTorqueConfig final {
   public:
    static RwMotorTorqueConfig create(const Eigen::Matrix3f& controlAxes_B) {
        if (!isValidControlAxes(controlAxes_B)) {
            FSW_THROW_INVALID_ARGUMENT(
                "rwMotorTorque: controlAxes_B must contain only finite values, define at least one control "
                "axis, and be filled from top to bottom with any zero (uncontrolled) axes at the bottom.");
        }
        return RwMotorTorqueConfig{controlAxes_B};
    }

    static bool isValidControlAxes(const Eigen::Matrix3f& controlAxes_B) {
        if (!controlAxes_B.allFinite()) {
            return false;
        }
        bool seenEmptyAxis = false;
        uint32_t numControlAxes = 0U;
        for (uint32_t i = 0U; i < 3U; ++i) {
            if (controlAxes_B.row(i).norm() > 0.0F) {
                if (seenEmptyAxis) {
                    return false;
                }
                numControlAxes += 1U;
            } else {
                seenEmptyAxis = true;
            }
        }
        return numControlAxes > 0U;
    }

    const Eigen::Matrix3f& getControlAxes() const { return this->controlAxes_B; }

   private:
    explicit RwMotorTorqueConfig(const Eigen::Matrix3f& controlAxes_B) : controlAxes_B(controlAxes_B) {}

    Eigen::Matrix3f controlAxes_B;
};

/*! @brief Top level structure for the sub-module routines. */
class RwMotorTorqueAlgorithm final {
   public:
    explicit RwMotorTorqueAlgorithm(const RwMotorTorqueConfig& config);

    void setConfig(const RwMotorTorqueConfig& config);
    void configure(const RwMotorTorqueArrayConfig& rwConfig,
                   const RwMotorTorqueAvailability& availability,
                   bool rwAvailIsLinked);
    Eigen::Vector<float, kMaxNumRw> update(const Eigen::Vector3f& Lr_B) const;  //!< [N-m] RW motor torques

   private:
    RwMotorTorqueConfig cfg;    //!< [-] validated configuration (control axes mapping matrix)
    uint32_t numControlAxes{};  //!< [-] counter indicating how many orthogonal axes are controlled
    uint32_t numAvailRW{};      //!< [-] number of reaction wheels available
    uint32_t numRW{};           //!< [-] number of reaction wheels on the vehicle
    Eigen::Matrix<float, 3, kMaxNumRw> CGs{
        Eigen::Matrix<float, 3, kMaxNumRw>::Zero()};                    //!< [-] The control mapping matrix [CB][G_s]
    std::array<FSWdeviceAvailability, kMaxNumRw> wheelsAvailability{};  //!< [-] Reaction wheel availability
};

#endif
