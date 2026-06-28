#ifndef F32XMERA_RW_MOTOR_TORQUE_ALGORITHM_H
#define F32XMERA_RW_MOTOR_TORQUE_ALGORITHM_H

#include "rwMotorTorqueTypes.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/freestandingIsFinite.hpp"
#include <fswAlgorithms/fswUtilities/fswDefinitions.h>
#include <math.h>

#include <Eigen/Core>
#include <array>
#include <cstdint>

inline constexpr uint32_t kMaxNumRw = RW_MOTOR_TORQUE_MAX_NUM_RW;

/*! @brief Reaction-wheel spin-axis configuration in body-frame components. */
struct RwMotorTorqueArrayConfiguration {
    uint32_t numRW{};  //!< [-] number of reaction wheels on the vehicle
    Eigen::Matrix<float, 3, kMaxNumRw> GsMatrix_B{
        Eigen::Matrix<float, 3, kMaxNumRw>::Zero()};  //!< [-] RW spin axes in body frame, one column per wheel
    std::array<FSWdeviceAvailability, kMaxNumRw>
        wheelAvailability{};  //!< [-] AVAILABLE / UNAVAILABLE state of each reaction wheel (fixed at reset)
};

/*! @brief Per-update reaction-wheel speeds for the null-space term. */
struct RwMotorTorqueSpeeds {
    Eigen::Vector<float, kMaxNumRw> rwSpeeds{Eigen::Vector<float, kMaxNumRw>::Zero()};  //!< [r/s] current RW speeds
    Eigen::Vector<float, kMaxNumRw> rwDesiredSpeeds{
        Eigen::Vector<float, kMaxNumRw>::Zero()};  //!< [r/s] desired RW speeds
};

/*!
 * @brief Validated configuration for the RW motor torque algorithm.
 *
 * Bundles the desired control-axis selection (which of body x, y, z to control), the reaction-wheel
 * spin-axis configuration, and the per-wheel availability. An instance can only exist if: at least one
 * control axis is selected; the reaction-wheel count does not exceed the compile-time maximum and each spin
 * axis is a unit vector; and the configuration yields a realizable, well-conditioned control mapping (every
 * selected control axis reachable by the available wheels, with neither the control mapping nor the
 * null-space geometry ill-conditioned). Construct via RwMotorTorqueConfig::create(...).
 */
class RwMotorTorqueConfig final {
   public:
    static RwMotorTorqueConfig create(const std::array<bool, 3>& desiredControlAxes_B,
                                      const RwMotorTorqueArrayConfiguration& rwConfiguration,
                                      float omegaGain = 0.0F) {
        if (!isValidControlAxes(desiredControlAxes_B)) {
            FSW_THROW_INVALID_ARGUMENT("rwMotorTorque: desiredControlAxes_B must select at least one control axis.");
        }
        if (!isValidRwConfiguration(rwConfiguration)) {
            FSW_THROW_INVALID_ARGUMENT(
                "rwMotorTorque: rwConfiguration.numRW must not exceed the compile-time maximum, the spin axis "
                "matrix must be finite, and each spin axis must be a unit vector.");
        }
        if (!isValidOmegaGain(omegaGain)) {
            FSW_THROW_INVALID_ARGUMENT(
                "rwMotorTorque: omegaGain (RW null-space feedback gain) must be finite and non-negative.");
        }

        // Normalize the RW spin axes so downstream code can rely on exact unit vectors. The inputs are validated
        // (near-)unit, so this only removes rounding.
        RwMotorTorqueArrayConfiguration normalizedRwConfiguration = rwConfiguration;
        for (uint32_t i = 0U; i < normalizedRwConfiguration.numRW; ++i) {
            normalizedRwConfiguration.GsMatrix_B.col(i).normalize();
        }

        // Reject configurations whose control mapping cannot be realized: a selected control axis is unreachable
        // by the available reaction wheels, or the control mapping or null-space geometry is ill-conditioned
        // (condition number above 100). This is the only place such an invalid argument is thrown -- once
        // constructed, a RwMotorTorqueConfig always yields a usable mapping.
        if (!isValidMapping(desiredControlAxes_B, normalizedRwConfiguration)) {
            FSW_THROW_INVALID_ARGUMENT(
                "rwMotorTorque: the configuration does not yield a valid control mapping -- a selected control "
                "axis is not reachable by the available reaction wheels, or the control mapping or null-space "
                "geometry is ill-conditioned.");
        }

        return RwMotorTorqueConfig{desiredControlAxes_B, normalizedRwConfiguration, omegaGain};
    }

    //! At least one body axis (x, y, z) must be selected for control.
    static bool isValidControlAxes(const std::array<bool, 3>& desiredControlAxes_B) {
        return desiredControlAxes_B[0] || desiredControlAxes_B[1] || desiredControlAxes_B[2];
    }

    static bool isValidRwConfiguration(const RwMotorTorqueArrayConfiguration& rwConfiguration) {
        if (rwConfiguration.numRW > kMaxNumRw || !rwConfiguration.GsMatrix_B.allFinite()) {
            return false;
        }
        // Each spin axis must be (close to) a unit vector.
        constexpr float kUnitNormTol = 1e-3F;
        for (uint32_t i = 0U; i < rwConfiguration.numRW; ++i) {
            if (fabsf(rwConfiguration.GsMatrix_B.col(i).norm() - 1.0F) > kUnitNormTol) {
                return false;
            }
        }
        return true;
    }
    // No isValidAvailability — any combination of AVAILABLE / UNAVAILABLE flags is valid.

    static bool isValidOmegaGain(float omegaGain) { return fsw::is_finite(omegaGain) && omegaGain >= 0.0F; }

    // Returns true if the (canonicalized) configuration yields a valid control mapping: every control axis is
    // reachable by the available reaction wheels, and both the control mapping and the null-space
    // geometry are well-conditioned (condition number below 100). Defined in the .cpp because it shares the
    // mapping computation with the algorithm.
    static bool isValidMapping(const std::array<bool, 3>& desiredControlAxes_B,
                               const RwMotorTorqueArrayConfiguration& rwConfiguration);

    const std::array<bool, 3>& getDesiredControlAxes() const { return this->desiredControlAxes_B; }
    const RwMotorTorqueArrayConfiguration& getRwConfiguration() const { return this->rwConfiguration; }
    float getOmegaGain() const { return this->omegaGain; }

   private:
    RwMotorTorqueConfig(const std::array<bool, 3>& desiredControlAxes_B,
                        const RwMotorTorqueArrayConfiguration& rwConfiguration,
                        float omegaGain)
        : desiredControlAxes_B(desiredControlAxes_B), rwConfiguration(rwConfiguration), omegaGain(omegaGain) {}

    std::array<bool, 3> desiredControlAxes_B;
    RwMotorTorqueArrayConfiguration rwConfiguration;
    float omegaGain;  //!< [-] RW null-space feedback gain (>= 0; 0 disables the null-space term)
};

/*! @brief Top level structure for the sub-module routines. */
class RwMotorTorqueAlgorithm final {
   public:
    explicit RwMotorTorqueAlgorithm(const RwMotorTorqueConfig& config);

    void setConfig(const RwMotorTorqueConfig& config);

    //! Control torque plus the RW null-space torque (pass zero speeds for control torque only).
    //! [N-m] RW motor torques
    Eigen::Vector<float, kMaxNumRw> update(const Eigen::Vector3f& Lr_B, const RwMotorTorqueSpeeds& speeds) const;

   private:
    RwMotorTorqueConfig cfg;  //!< [-] validated configuration (control axes, RW config, availability, gain)
    Eigen::Matrix<float, kMaxNumRw, 3> motorTorqueMap{
        Eigen::Matrix<float, kMaxNumRw, 3>::Zero()};  //!< [-] maps the commanded body control torque to per-RW
                                                      //!< motor torques (rows of unavailable wheels are zero)
    Eigen::Matrix<float, kMaxNumRw, kMaxNumRw> tau{
        Eigen::Matrix<float, kMaxNumRw, kMaxNumRw>::Zero()};  //!< [-] RW null-space projection matrix (zero when
                                                              //!< the wheels do not span 3-D, disabling the null-space
                                                              //!< term)
};

#endif
