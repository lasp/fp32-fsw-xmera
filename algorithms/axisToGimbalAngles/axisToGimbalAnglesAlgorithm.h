#ifndef F32XMERA_AXIS_TO_GIMBAL_ANGLES_ALGORITHM_H
#define F32XMERA_AXIS_TO_GIMBAL_ANGLES_ALGORITHM_H

#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/rigidBodyKinematics.hpp"

#include <Eigen/Core>

/*! @brief Gimbal angles that place the gimbal thrust axis on the requested direction. */
struct AxisToGimbalAnglesOutput {
    float gimbalAngle1{};  //!< [rad] alpha: inclination of the thrust axis projected into the mount y-z plane
    float gimbalAngle2{};  //!< [rad] beta: inclination of the thrust axis projected into the mount x-z plane
};

/*!
 * @brief Validated configuration for the axis to gimbal angles algorithm.
 *
 * Carries the only quantity the mapping needs: the orientation of the gimbal mount frame M on the hub. The M
 * frame is defined with its -z axis along the un-deflected thrust axis, so a neutral gimbal fires along -z_M.
 * Construct via AxisToGimbalAnglesConfig::create(...).
 */
class AxisToGimbalAnglesConfig final {
   public:
    static AxisToGimbalAnglesConfig create(const Eigen::Vector3f& sigma_MB) {
        if (!isValidSigma_MB(sigma_MB)) {
            FSW_THROW_INVALID_ARGUMENT("axisToGimbalAngles: sigma_MB must be finite.");
        }

        // The shadow set describes the same rotation, so store the principal one (norm <= 1).
        return AxisToGimbalAnglesConfig{mrpSwitch(sigma_MB)};
    }

    static bool isValidSigma_MB(const Eigen::Vector3f& sigma_MB) { return sigma_MB.allFinite(); }

    const Eigen::Vector3f& getSigma_MB() const { return this->sigma_MB; }

   private:
    // NOLINTBEGIN(modernize-pass-by-value)
    // modernize-pass-by-value: this is a private constructor invoked only from create() with an already-validated
    //   argument; the small fixed-size vector is stored by copy without a move for clarity.
    explicit AxisToGimbalAnglesConfig(const Eigen::Vector3f& sigma_MB) : sigma_MB(sigma_MB) {}
    // NOLINTEND(modernize-pass-by-value)

    Eigen::Vector3f sigma_MB;  //!< [-] MRP of the mount frame M w.r.t. the body frame B
};

/*! @brief Pure algorithm mapping a commanded body-frame thrust direction onto the two gimbal angles.
 *
 * The gimbal is described by two independent plane angles, each measured on the thrust axis projected into one of
 * mount planes that contain the un-deflected axis, so the map inverts in closed form as a pair of arctangents.
 * The algorithm holds no runtime state.
 */
class AxisToGimbalAnglesAlgorithm final {
   public:
    explicit AxisToGimbalAnglesAlgorithm(const AxisToGimbalAnglesConfig& config);
    void setConfig(const AxisToGimbalAnglesConfig& config);
    AxisToGimbalAnglesOutput update(const Eigen::Vector3f& thrustHat_B) const;

   private:
    AxisToGimbalAnglesConfig cfg;  //!< [-] validated configuration
    //! [-] DCM from the body frame to the mount frame, resolved from sigma_MB whenever the configuration is set
    //! so the per-cycle map never has to rebuild it
    Eigen::Matrix3f dcm_MB{Eigen::Matrix3f::Identity()};
};

#endif  // F32XMERA_AXIS_TO_GIMBAL_ANGLES_ALGORITHM_H
