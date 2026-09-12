#ifndef F32XMERA_AXIS_TO_GIMBAL_ANGLES_ALGORITHM_H
#define F32XMERA_AXIS_TO_GIMBAL_ANGLES_ALGORITHM_H

#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/freestandingIsFinite.hpp"
#include "utilities/fsw/rigidBodyKinematics.hpp"

#include <Eigen/Core>
#include <numbers>

/*! @brief Gimbal angles that place the gimbal thrust axis on the requested direction. */
struct AxisToGimbalAnglesOutput {
    float gimbalAngle1{};  //!< [rad] alpha: inclination of the thrust axis projected into the mount y-z plane
    float gimbalAngle2{};  //!< [rad] beta: inclination of the thrust axis projected into the mount x-z plane
};

/*!
 * @brief Validated configuration for the axis to gimbal angles algorithm.
 *
 * Carries the orientation of the gimbal mount frame M on the hub and the travel of the mechanism. The M frame is
 * defined with its +z axis along the un-deflected thrust axis, so a neutral gimbal fires along +z_M. Construct
 * via AxisToGimbalAnglesConfig::create(...).
 */
class AxisToGimbalAnglesConfig final {
   public:
    static AxisToGimbalAnglesConfig create(const Eigen::Vector3f& sigma_MB, float thetaMax) {
        if (!isValidSigma_MB(sigma_MB)) {
            FSW_THROW_INVALID_ARGUMENT("axisToGimbalAngles: sigma_MB must be finite.");
        }
        if (!isValidThetaMax(thetaMax)) {
            FSW_THROW_INVALID_ARGUMENT(
                "axisToGimbalAngles: thetaMax must lie in the open interval (0, pi/2). Two plane angles cannot "
                "describe a deflection of 90 degrees or more.");
        }

        // The shadow set describes the same rotation, so store the principal one (norm <= 1).
        return AxisToGimbalAnglesConfig{mrpSwitch(sigma_MB), thetaMax};
    }

    static bool isValidSigma_MB(const Eigen::Vector3f& sigma_MB) { return sigma_MB.allFinite(); }
    /*! Each angle is an arctangent of a ratio against the mount +z axis, which goes to infinity at a deflection
     * of 90 degrees. A travel below that keeps both angles bounded by thetaMax itself. */
    static bool isValidThetaMax(float thetaMax) {
        return fsw::is_finite(thetaMax) && thetaMax > 0.0F && thetaMax < (std::numbers::pi_v<float> / 2.0F);
    }

    const Eigen::Vector3f& getSigma_MB() const { return this->sigma_MB; }
    float getThetaMax() const { return this->thetaMax; }

   private:
    // NOLINTBEGIN(modernize-pass-by-value)
    // modernize-pass-by-value: this is a private constructor invoked only from create() with an already-validated
    //   argument; the small fixed-size vector is stored by copy without a move for clarity.
    AxisToGimbalAnglesConfig(const Eigen::Vector3f& sigma_MB, float thetaMax)
        : sigma_MB(sigma_MB), thetaMax(thetaMax) {}
    // NOLINTEND(modernize-pass-by-value)

    Eigen::Vector3f sigma_MB;  //!< [-] MRP of the mount frame M w.r.t. the body frame B
    float thetaMax;            //!< [rad] largest deflection of the thrust axis from the neutral axis
};

/*! @brief Pure algorithm mapping a commanded body-frame thrust direction onto the two gimbal angles.
 *
 * The gimbal is described by two independent plane angles, each measured on the thrust axis projected into one of
 * mount planes that contain the un-deflected axis, so the map inverts in closed form as a pair of arctangents. A
 * request beyond the travel of the mechanism is first pulled back onto the cone of half-angle thetaMax, which
 * keeps both angles inside that same bound. The algorithm holds no runtime state.
 */
class AxisToGimbalAnglesAlgorithm final {
   public:
    explicit AxisToGimbalAnglesAlgorithm(const AxisToGimbalAnglesConfig& config);
    void setConfig(const AxisToGimbalAnglesConfig& config);
    AxisToGimbalAnglesOutput update(const Eigen::Vector3f& thrustHat_B) const;

   private:
    Eigen::Vector3f clampDeflection(const Eigen::Vector3f& thrustHat_M) const;

    AxisToGimbalAnglesConfig cfg;  //!< [-] validated configuration
    //! Resolved from the configuration whenever it is set, so the per-cycle map never has to rebuild them. The
    //! only constructor sets all three, thus the values below are never the ones a call to update() sees.
    Eigen::Matrix3f dcm_MB{Eigen::Matrix3f::Zero()};  //!< [-] DCM from the body frame to the mount frame
    float cosThetaMax{};                              //!< [-] cosine of the largest deflection
    float sinThetaMax{};                              //!< [-] sine of the largest deflection
};

#endif  // F32XMERA_AXIS_TO_GIMBAL_ANGLES_ALGORITHM_H
