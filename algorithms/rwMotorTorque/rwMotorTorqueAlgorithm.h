#ifndef F32XMERA_RW_MOTOR_TORQUE_ALGORITHM_H
#define F32XMERA_RW_MOTOR_TORQUE_ALGORITHM_H

#include "rwMotorTorqueTypes.h"
#include "utilities/freestandingInvalidArgument.h"
#include "utilities/freestandingIsFinite.hpp"
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
};

/*! @brief Per-wheel reaction-wheel availability. */
struct RwMotorTorqueAvailability {
    std::array<FSWdeviceAvailability, kMaxNumRw>
        wheelAvailability{};  //!< [-] AVAILABLE / UNAVAILABLE state of each reaction wheel
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
 * Bundles the control axes mapping matrix, the reaction-wheel spin-axis configuration, and the per-wheel
 * availability. An instance can only exist if: the control axes mapping matrix is finite and defines at least
 * one control axis (each non-zero row a unit vector, the non-zero rows mutually orthogonal, zero/uncontrolled
 * rows allowed in any position); the reaction-wheel count does not exceed the compile-time maximum and each
 * spin axis is a unit vector; and the configuration yields a realizable, well-conditioned control mapping
 * (every control axis reachable by the available wheels, with neither the control mapping nor the null-space
 * null-space geometry ill-conditioned). Construct via RwMotorTorqueConfig::create(...).
 */
class RwMotorTorqueConfig final {
   public:
    static RwMotorTorqueConfig create(const Eigen::Matrix3f& controlAxes_B,
                                      const RwMotorTorqueArrayConfiguration& rwConfiguration,
                                      const RwMotorTorqueAvailability& availability,
                                      float omegaGain = 0.0F) {
        if (!isValidControlAxes(controlAxes_B)) {
            FSW_THROW_INVALID_ARGUMENT(
                "rwMotorTorque: controlAxes_B must contain only finite values and define at least one control "
                "axis, each non-zero row a unit vector and the non-zero rows mutually orthogonal (zero rows mark "
                "uncontrolled axes).");
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

        // Canonicalize the validated inputs so downstream code can rely on exact unit vectors. The control
        // axes are orthonormalized (Gram-Schmidt over the non-zero rows; zero/uncontrolled rows stay in
        // place), and the RW spin axes are normalized. The inputs are validated (near-)unit and (near-)
        // orthogonal, so this only removes rounding.
        Eigen::Matrix3f orthonormalControlAxes = controlAxes_B;
        for (uint32_t i = 0U; i < 3U; ++i) {
            if (orthonormalControlAxes.row(i).norm() <= 0.0F) {
                continue;
            }
            for (uint32_t k = 0U; k < i; ++k) {
                if (orthonormalControlAxes.row(k).norm() > 0.0F) {
                    orthonormalControlAxes.row(i) -= orthonormalControlAxes.row(i).dot(orthonormalControlAxes.row(k)) *
                                                     orthonormalControlAxes.row(k);
                }
            }
            orthonormalControlAxes.row(i).normalize();
        }
        RwMotorTorqueArrayConfiguration normalizedRwConfiguration = rwConfiguration;
        for (uint32_t i = 0U; i < normalizedRwConfiguration.numRW; ++i) {
            normalizedRwConfiguration.GsMatrix_B.col(i).normalize();
        }

        // Reject configurations whose control mapping cannot be realized: a control axis is unreachable by the
        // available reaction wheels, or the control mapping or null-space geometry is ill-conditioned
        // (condition number above 100). This is the only place such an invalid argument is thrown -- once
        // constructed, a RwMotorTorqueConfig always yields a usable mapping.
        if (!isValidMapping(orthonormalControlAxes, normalizedRwConfiguration, availability)) {
            FSW_THROW_INVALID_ARGUMENT(
                "rwMotorTorque: the configuration does not yield a valid control mapping -- a control axis is not "
                "reachable by the available reaction wheels, or the control mapping or null-space "
                "geometry is ill-conditioned.");
        }

        return RwMotorTorqueConfig{orthonormalControlAxes, normalizedRwConfiguration, availability, omegaGain};
    }

    static bool isValidControlAxes(const Eigen::Matrix3f& controlAxes_B) {
        if (!controlAxes_B.allFinite()) {
            return false;
        }
        // Each non-zero row is a control axis (in any position); the control axes must be unit vectors
        // and mutually orthogonal. Zero rows mark uncontrolled body directions.
        constexpr float kOrthonormalTol = 1e-3F;
        uint32_t numControlAxes = 0U;
        for (uint32_t i = 0U; i < 3U; ++i) {
            if (controlAxes_B.row(i).norm() <= 0.0F) {
                continue;
            }
            numControlAxes += 1U;
            if (fabsf(controlAxes_B.row(i).norm() - 1.0F) > kOrthonormalTol) {
                return false;
            }
            for (uint32_t k = i + 1U; k < 3U; ++k) {
                if (controlAxes_B.row(k).norm() > 0.0F &&
                    fabsf(controlAxes_B.row(i).dot(controlAxes_B.row(k))) > kOrthonormalTol) {
                    return false;
                }
            }
        }
        return numControlAxes > 0U;
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
    static bool isValidMapping(const Eigen::Matrix3f& controlAxes_B,
                               const RwMotorTorqueArrayConfiguration& rwConfiguration,
                               const RwMotorTorqueAvailability& availability);

    const Eigen::Matrix3f& getControlAxes() const { return this->controlAxes_B; }
    const RwMotorTorqueArrayConfiguration& getRwConfiguration() const { return this->rwConfiguration; }
    const RwMotorTorqueAvailability& getAvailability() const { return this->availability; }
    float getOmegaGain() const { return this->omegaGain; }

   private:
    RwMotorTorqueConfig(const Eigen::Matrix3f& controlAxes_B,
                        const RwMotorTorqueArrayConfiguration& rwConfiguration,
                        const RwMotorTorqueAvailability& availability,
                        float omegaGain)
        : controlAxes_B(controlAxes_B),
          rwConfiguration(rwConfiguration),
          availability(availability),
          omegaGain(omegaGain) {}

    Eigen::Matrix3f controlAxes_B;
    RwMotorTorqueArrayConfiguration rwConfiguration;
    RwMotorTorqueAvailability availability;
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
