#ifndef F32XMERA_THRUST_VECTORING_ALGORITHM_H
#define F32XMERA_THRUST_VECTORING_ALGORITHM_H

#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/freestandingIsFinite.hpp"
#include "utilities/fsw/rigidBodyKinematics.hpp"

#include <Eigen/Core>
#include <numbers>

//! [m] smallest center-of-mass offset from the platform joint M for which a reference pointing is defined
inline constexpr float kMinR_CM = 1e-3F;

/*! @brief Mounting geometry of the platform on the hub and the limit on how far it may deflect the thrust.
 *
 * The mount frame M is defined with its -z axis along the un-deflected thrust direction, so sigma_MB carries the
 * thruster's mounting orientation on the hub.
 */
struct ThrustVectoringPlatformConfiguration {
    Eigen::Vector3f sigma_MB{Eigen::Vector3f::Zero()};  //!< [-] MRP of the M frame w.r.t. the B frame
    Eigen::Vector3f r_MB_B{Eigen::Vector3f::Zero()};    //!< [m] M frame origin w.r.t. B frame origin, B frame
    float thetaMax{};                                   //!< [rad] half-angle of the thrust-deflection cone
};

/*! @brief Thruster configuration
 *
 * The thrust acts along the platform's -z axis and is applied armLength behind the pivot M along that axis, so
 * its line of action runs through M whatever the platform orientation.
 */
struct ThrustVectoringThrusterConfiguration {
    float armLength{};  //!< [m] distance from the pivot M to the thrust application point, along the thrust
    float thrust{};     //!< [N] thrust magnitude
};

/*! @brief Outputs of the thrust vectoring algorithm. */
struct ThrustVectoringOutput {
    Eigen::Vector3f r_TB_B{Eigen::Vector3f::Zero()};  //!< [m] thrust application point w.r.t. B origin, B frame
    Eigen::Vector3f tHat_B{Eigen::Vector3f::Zero()};  //!< [-] thrust unit direction, B frame
    float thrust{};                                   //!< [N] thrust magnitude
};

/*!
 * @brief Validated configuration for the thrust vectoring algorithm.
 *
 * Bundles the platform mounting geometry, the thruster geometry and the center-of-mass position, all of which are
 * fixed while the module runs. An instance can only exist if the mounting vectors are finite, the deflection cone
 * half-angle lies in (0, pi), the arm length is finite and non-negative, the thrust magnitude is finite and
 * positive, and the center of mass is far enough from the platform joint for the pointing to be defined.
 * Construct via ThrustVectoringConfig::create(...).
 */
class ThrustVectoringConfig final {
   public:
    static ThrustVectoringConfig create(const ThrustVectoringPlatformConfiguration& platformConfig,
                                        const ThrustVectoringThrusterConfiguration& thrusterConfig,
                                        const Eigen::Vector3f& r_CB_B) {
        if (!isValidSigma_MB(platformConfig.sigma_MB)) {
            FSW_THROW_INVALID_ARGUMENT("thrustVectoring: sigma_MB must be finite.");
        }
        if (!isValidR_MB_B(platformConfig.r_MB_B)) {
            FSW_THROW_INVALID_ARGUMENT("thrustVectoring: r_MB_B must be finite.");
        }
        if (!isValidThetaMax(platformConfig.thetaMax)) {
            FSW_THROW_INVALID_ARGUMENT("thrustVectoring: thetaMax must lie in the open interval (0, pi).");
        }
        if (!isValidArmLength(thrusterConfig.armLength)) {
            FSW_THROW_INVALID_ARGUMENT("thrustVectoring: armLength must be finite and non-negative.");
        }
        if (!isValidThrust(thrusterConfig.thrust)) {
            FSW_THROW_INVALID_ARGUMENT("thrustVectoring: thrust must be finite and positive.");
        }
        if (!isValidR_CB_B(r_CB_B)) {
            FSW_THROW_INVALID_ARGUMENT("thrustVectoring: r_CB_B must be finite.");
        }
        if (!isValidR_CM(r_CB_B, platformConfig.r_MB_B)) {
            FSW_THROW_INVALID_ARGUMENT(
                "thrustVectoring: the center of mass must be farther than kMinR_CM from the platform joint M, "
                "otherwise no reference pointing is defined.");
        }
        if (!isThrustDirectedInboard(platformConfig, r_CB_B)) {
            FSW_THROW_INVALID_ARGUMENT(
                "thrustVectoring: the un-deflected thrust must point from the platform joint M towards the center "
                "of mass, otherwise the thruster is mounted inboard of the joint, inside the vehicle.");
        }

        // Bound sigma_MB to the principal MRP set (norm <= 1) by switching to the shadow set if needed, so the
        // stored orientation is always a well-conditioned MRP representation.
        ThrustVectoringPlatformConfiguration boundedPlatformConfig = platformConfig;
        boundedPlatformConfig.sigma_MB = mrpSwitch(platformConfig.sigma_MB);

        return {boundedPlatformConfig, thrusterConfig, r_CB_B};
    }

    static bool isValidSigma_MB(const Eigen::Vector3f& sigma_MB) { return sigma_MB.allFinite(); }
    static bool isValidR_MB_B(const Eigen::Vector3f& r_MB_B) { return r_MB_B.allFinite(); }
    static bool isValidThetaMax(float thetaMax) {
        return fsw::is_finite(thetaMax) && thetaMax > 0.0F && thetaMax < std::numbers::pi_v<float>;
    }
    static bool isValidArmLength(float armLength) { return fsw::is_finite(armLength) && armLength >= 0.0F; }
    /*! A zero thrust leaves the thruster with no torque authority and no defined line of action. */
    static bool isValidThrust(float thrust) { return fsw::is_finite(thrust) && thrust > 0.0F; }
    static bool isValidR_CB_B(const Eigen::Vector3f& r_CB_B) { return r_CB_B.allFinite(); }
    static bool isValidR_CM(const Eigen::Vector3f& r_CB_B, const Eigen::Vector3f& r_MB_B) {
        return (r_CB_B - r_MB_B).norm() > kMinR_CM;
    }
    /*! The thruster is mounted armLength behind the joint along the thrust, so the un-deflected thrust must point
     * from the joint back towards the center of mass for the thruster itself to sit outboard of the joint rather
     * than inside the vehicle. A deflection stays on the correct side of the joint as long as the cone does not
     * reach a right angle. Call only for a geometry isValidR_CM has already accepted, so r_MC has a direction. */
    static bool isThrustDirectedInboard(const ThrustVectoringPlatformConfiguration& platformConfig,
                                        const Eigen::Vector3f& r_CB_B) {
        const Eigen::Vector3f tHatNeutral_B =
            -mrpToDcm(mrpSwitch(platformConfig.sigma_MB)).row(2).transpose().normalized();
        return (platformConfig.r_MB_B - r_CB_B).stableNormalized().dot(tHatNeutral_B) < 0.0F;
    }

    const ThrustVectoringPlatformConfiguration& getPlatformConfiguration() const { return this->platformConfig; }
    const ThrustVectoringThrusterConfiguration& getThrusterConfiguration() const { return this->thrusterConfig; }
    const Eigen::Vector3f& getR_CB_B() const { return this->r_CB_B; }

   private:
    // NOLINTBEGIN(modernize-pass-by-value)
    // modernize-pass-by-value: this is a private constructor invoked only from create() with already-validated
    //   arguments; the small configuration structs are stored by copy without a move for clarity.
    ThrustVectoringConfig(const ThrustVectoringPlatformConfiguration& platformConfig,
                          const ThrustVectoringThrusterConfiguration& thrusterConfig,
                          const Eigen::Vector3f& r_CB_B)
        : platformConfig(platformConfig), thrusterConfig(thrusterConfig), r_CB_B(r_CB_B) {}
    // NOLINTEND(modernize-pass-by-value)

    ThrustVectoringPlatformConfiguration platformConfig;  //!< [-] platform mounting geometry and cone limit
    ThrustVectoringThrusterConfiguration thrusterConfig;  //!< [-] thruster arm length and magnitude
    Eigen::Vector3f r_CB_B;                               //!< [m] center of mass w.r.t. B origin, B frame
};

/*! @brief Pure algorithm computing the reference orientation of a thruster platform.
 *
 * The thrust line of action runs through the platform joint, which makes the torque it produces depend on the
 * platform only through the thrust direction. The reference is therefore a closed-form solve with no state and
 * no dependence on the previous cycle.
 */
class ThrustVectoringAlgorithm final {
   public:
    explicit ThrustVectoringAlgorithm(const ThrustVectoringConfig& config);
    void setConfig(const ThrustVectoringConfig& config);
    ThrustVectoringOutput update(const Eigen::Vector3f& Lreq_B) const;

   private:
    ThrustVectoringConfig cfg;  //!< [-] validated configuration
    //! [-] un-deflected thrust direction, body frame: the mount frame's -z axis, resolved from sigma_MB whenever
    //! the configuration is set, so the per-cycle solve never has to rotate anything
    Eigen::Vector3f tHatNeutral_B{-Eigen::Vector3f::UnitZ()};
};

#endif  // F32XMERA_THRUST_VECTORING_ALGORITHM_H
