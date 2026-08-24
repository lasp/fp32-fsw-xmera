#ifndef F32XMERA_THRUST_VECTORING_ALGORITHM_H
#define F32XMERA_THRUST_VECTORING_ALGORITHM_H

#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/freestandingIsFinite.hpp"
#include "utilities/fsw/rigidBodyKinematics.hpp"
#include <math.h>

#include <Eigen/Core>
#include <numbers>

/*! @brief Mounting geometry of the platform on the hub and the limit on how far it may deflect the thrust. */
struct ThrustVectoringPlatformConfiguration {
    Eigen::Vector3f sigma_MB{Eigen::Vector3f::Zero()};  //!< [-] MRP of the M frame w.r.t. the B frame
    Eigen::Vector3f r_MB_B{Eigen::Vector3f::Zero()};    //!< [m] M frame origin w.r.t. B frame origin, B frame
    Eigen::Vector3f r_FM_F{Eigen::Vector3f::Zero()};    //!< [m] F frame origin w.r.t. M frame origin, F frame
    float thetaMax{};                                   //!< [rad] half-angle of the thrust-deflection cone
};

/*! @brief Thruster geometry and magnitude, fixed in the platform frame. */
struct ThrustVectoringThrusterConfiguration {
    Eigen::Vector3f r_TF_F{Eigen::Vector3f::Zero()};  //!< [m] thrust application point w.r.t. F origin, F frame
    Eigen::Vector3f tHat_F{Eigen::Vector3f::Zero()};  //!< [-] thrust unit direction, F frame
    float thrust{};                                   //!< [N] thrust magnitude
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
 * fixed while the module runs. An instance can only exist if every position vector is finite, the deflection cone
 * half-angle lies in (0, pi), the thrust direction is a (near-)unit vector and the thrust magnitude is finite and
 * positive. Construct via ThrustVectoringConfig::create(...).
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
        if (!isValidR_FM_F(platformConfig.r_FM_F)) {
            FSW_THROW_INVALID_ARGUMENT("thrustVectoring: r_FM_F must be finite.");
        }
        if (!isValidThetaMax(platformConfig.thetaMax)) {
            FSW_THROW_INVALID_ARGUMENT("thrustVectoring: thetaMax must lie in the open interval (0, pi).");
        }
        if (!isValidR_TF_F(thrusterConfig.r_TF_F)) {
            FSW_THROW_INVALID_ARGUMENT("thrustVectoring: r_TF_F must be finite.");
        }
        if (!isValidTHat_F(thrusterConfig.tHat_F)) {
            FSW_THROW_INVALID_ARGUMENT("thrustVectoring: tHat_F must be a (close to) unit vector.");
        }
        if (!isValidThrust(thrusterConfig.thrust)) {
            FSW_THROW_INVALID_ARGUMENT("thrustVectoring: thrust must be finite and positive.");
        }
        if (!isValidR_CB_B(r_CB_B)) {
            FSW_THROW_INVALID_ARGUMENT("thrustVectoring: r_CB_B must be finite.");
        }

        // Bound sigma_MB to the principal MRP set (norm <= 1) by switching to the shadow set if needed, so the
        // stored orientation is always a well-conditioned MRP representation.
        ThrustVectoringPlatformConfiguration boundedPlatformConfig = platformConfig;
        boundedPlatformConfig.sigma_MB = mrpSwitch(platformConfig.sigma_MB);

        // Normalize the thrust direction so the pointing solve can rely on an exact unit vector. The input is
        // validated (near-)unit, so this only removes rounding.
        ThrustVectoringThrusterConfiguration normalizedThrusterConfig = thrusterConfig;
        normalizedThrusterConfig.tHat_F.normalize();

        return {boundedPlatformConfig, normalizedThrusterConfig, r_CB_B};
    }

    static bool isValidSigma_MB(const Eigen::Vector3f& sigma_MB) { return sigma_MB.allFinite(); }
    static bool isValidR_MB_B(const Eigen::Vector3f& r_MB_B) { return r_MB_B.allFinite(); }
    static bool isValidR_FM_F(const Eigen::Vector3f& r_FM_F) { return r_FM_F.allFinite(); }
    static bool isValidThetaMax(float thetaMax) {
        return fsw::is_finite(thetaMax) && thetaMax > 0.0F && thetaMax < std::numbers::pi_v<float>;
    }
    static bool isValidR_TF_F(const Eigen::Vector3f& r_TF_F) { return r_TF_F.allFinite(); }
    /*! The thrust direction must be (close to) a unit vector; it is normalized exactly on construction. */
    static bool isValidTHat_F(const Eigen::Vector3f& tHat_F) {
        constexpr float kUnitNormTol = 1e-3F;
        return tHat_F.allFinite() && fabsf(tHat_F.norm() - 1.0F) <= kUnitNormTol;
    }
    /*! A zero thrust leaves the thruster with no torque authority and no defined line of action. */
    static bool isValidThrust(float thrust) { return fsw::is_finite(thrust) && thrust > 0.0F; }
    static bool isValidR_CB_B(const Eigen::Vector3f& r_CB_B) { return r_CB_B.allFinite(); }

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
    ThrustVectoringThrusterConfiguration thrusterConfig;  //!< [-] thruster geometry and magnitude
    Eigen::Vector3f r_CB_B;                               //!< [m] center of mass w.r.t. B origin, B frame
};

/*! @brief Pure algorithm computing the reference orientation of a thruster platform. */
class ThrustVectoringAlgorithm final {
   public:
    explicit ThrustVectoringAlgorithm(const ThrustVectoringConfig& config);
    void setConfig(const ThrustVectoringConfig& config);
    void reInitialize();
    ThrustVectoringOutput update(const Eigen::Vector3f& Lreq_B);

   private:
    /*! Convert the requested body-frame torque into platform-frame coordinates, seeding the conversion on the
     first cycle with the nominal zero-torque pointing. */
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) -- the vectors are distinct by frame and documented.
    Eigen::Vector3f torqueInPlatformFrame(const Eigen::Vector3f& Lreq_B,
                                          const Eigen::Vector3f& r_CM_M,
                                          const Eigen::Vector3f& r_TM_F,
                                          const Eigen::Vector3f& thrust_F,
                                          const Eigen::Matrix3f& dcm_MB);

    ThrustVectoringConfig cfg;  //!< [-] validated configuration
    Eigen::Matrix3f priorDcm_FM{
        Eigen::Matrix3f::Zero()};  //!< [-] previous cycle's reference DCM [FM] (torque-conversion seed; zero if unset)
};

#endif  // F32XMERA_THRUST_VECTORING_ALGORITHM_H
