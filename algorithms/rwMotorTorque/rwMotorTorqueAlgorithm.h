#ifndef F32XMERA_RW_MOTOR_TORQUE_ALGORITHM_H
#define F32XMERA_RW_MOTOR_TORQUE_ALGORITHM_H

#include "rwMotorTorqueTypes.h"
#include "utilities/freestandingInvalidArgument.h"
#include <fswAlgorithms/fswUtilities/fswDefinitions.h>

#include <Eigen/Core>
#include <array>
#include <cstdint>

inline constexpr uint32_t kMaxNumRw = RW_MOTOR_TORQUE_MAX_NUM_RW;

/*! @brief Reaction-wheel spin-axis configuration in body-frame components. */
struct RwMotorTorqueArrayConfiguration {
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
 * Bundles the control axes mapping matrix, the reaction-wheel spin-axis configuration, and the
 * per-wheel availability. An instance can only exist if the control axes mapping matrix is finite,
 * defines at least one control axis, and is filled from top to bottom (any zero, uncontrolled axes
 * at the bottom), and the reaction-wheel count does not exceed the compile-time maximum. Construct
 * via RwMotorTorqueConfig::create(...).
 */
class RwMotorTorqueConfig final {
   public:
    static RwMotorTorqueConfig create(const Eigen::Matrix3f& controlAxes_B,
                                      const RwMotorTorqueArrayConfiguration& rwConfiguration,
                                      const RwMotorTorqueAvailability& availability) {
        if (!isValidControlAxes(controlAxes_B)) {
            FSW_THROW_INVALID_ARGUMENT(
                "rwMotorTorque: controlAxes_B must contain only finite values, define at least one control "
                "axis, and be filled from top to bottom with any zero (uncontrolled) axes at the bottom.");
        }
        if (!isValidRwConfiguration(rwConfiguration)) {
            FSW_THROW_INVALID_ARGUMENT(
                "rwMotorTorque: rwConfiguration.numRW must not exceed the compile-time maximum and the spin "
                "axis matrix must be finite.");
        }
        return RwMotorTorqueConfig{controlAxes_B, rwConfiguration, availability};
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

    static bool isValidRwConfiguration(const RwMotorTorqueArrayConfiguration& rwConfiguration) {
        return rwConfiguration.numRW <= kMaxNumRw && rwConfiguration.GsMatrix_B.allFinite();
    }
    // No isValidAvailability — any combination of AVAILABLE / UNAVAILABLE flags is valid.

    const Eigen::Matrix3f& getControlAxes() const { return this->controlAxes_B; }
    const RwMotorTorqueArrayConfiguration& getRwConfiguration() const { return this->rwConfiguration; }
    const RwMotorTorqueAvailability& getAvailability() const { return this->availability; }

   private:
    RwMotorTorqueConfig(const Eigen::Matrix3f& controlAxes_B,
                        const RwMotorTorqueArrayConfiguration& rwConfiguration,
                        const RwMotorTorqueAvailability& availability)
        : controlAxes_B(controlAxes_B), rwConfiguration(rwConfiguration), availability(availability) {}

    Eigen::Matrix3f controlAxes_B;
    RwMotorTorqueArrayConfiguration rwConfiguration;
    RwMotorTorqueAvailability availability;
};

/*! @brief Top level structure for the sub-module routines. */
class RwMotorTorqueAlgorithm final {
   public:
    explicit RwMotorTorqueAlgorithm(const RwMotorTorqueConfig& config);

    void setConfig(const RwMotorTorqueConfig& config);
    Eigen::Vector<float, kMaxNumRw> update(const Eigen::Vector3f& Lr_B) const;  //!< [N-m] RW motor torques

   private:
    void computeRwMapping();  //!< builds motorTorqueMap from cfg; throws if the mapping is not full rank

    RwMotorTorqueConfig cfg;  //!< [-] validated configuration (control axes, RW config, availability)
    Eigen::Matrix<float, kMaxNumRw, 3> motorTorqueMap{
        Eigen::Matrix<float, kMaxNumRw, 3>::Zero()};  //!< [-] maps the commanded body control torque to per-RW
                                                      //!< motor torques (rows of unavailable wheels are zero)
};

#endif
