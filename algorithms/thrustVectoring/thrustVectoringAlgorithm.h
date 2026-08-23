#ifndef F32XMERA_THRUST_VECTORING_ALGORITHM_H
#define F32XMERA_THRUST_VECTORING_ALGORITHM_H

#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/freestandingIsFinite.hpp"
#include "utilities/fsw/rigidBodyKinematics.hpp"

#include <Eigen/Core>
#include <numbers>

/*! @brief Per-cycle inputs to the thrust vectoring algorithm. */
struct ThrustVectoringInputs {
    Eigen::Vector3f r_CB_B{Eigen::Vector3f::Zero()};  //!< [m] center of mass w.r.t. B origin, B frame
    Eigen::Vector3f r_TF_F{Eigen::Vector3f::Zero()};  //!< [m] thrust application point w.r.t. F origin, F frame
    Eigen::Vector3f tHat_F{Eigen::Vector3f::Zero()};  //!< [-] thrust unit direction, F frame
    float thrust{};                                   //!< [N] thrust magnitude
    Eigen::Vector3f Lreq_B{
        Eigen::Vector3f::Zero()};  //!< [Nm] requested thruster torque about the center of mass, B frame
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
 * Bundles the platform mounting geometry and the thrust-deflection cone limit. An instance can only exist if the
 * geometry vectors are finite and the deflection cone half-angle lies in (0, pi). Construct via
 * ThrustVectoringConfig::create(...).
 */
class ThrustVectoringConfig final {
   public:
    static ThrustVectoringConfig create(const Eigen::Vector3f& sigma_MB,
                                        const Eigen::Vector3f& r_MB_B,
                                        const Eigen::Vector3f& r_FM_F,
                                        float thetaMax) {
        if (!isValidSigma_MB(sigma_MB)) {
            FSW_THROW_INVALID_ARGUMENT("thrustVectoring: sigma_MB must be finite.");
        }
        if (!isValidR_MB_B(r_MB_B)) {
            FSW_THROW_INVALID_ARGUMENT("thrustVectoring: r_MB_B must be finite.");
        }
        if (!isValidR_FM_F(r_FM_F)) {
            FSW_THROW_INVALID_ARGUMENT("thrustVectoring: r_FM_F must be finite.");
        }
        if (!isValidThetaMax(thetaMax)) {
            FSW_THROW_INVALID_ARGUMENT("thrustVectoring: thetaMax must lie in the open interval (0, pi).");
        }

        // Bound sigma_MB to the principal MRP set (norm <= 1) by switching to the shadow set if needed, so the
        // stored orientation is always a well-conditioned MRP representation.
        return {mrpSwitch(sigma_MB), r_MB_B, r_FM_F, thetaMax};
    }

    static bool isValidSigma_MB(const Eigen::Vector3f& sigma_MB) { return sigma_MB.allFinite(); }
    static bool isValidR_MB_B(const Eigen::Vector3f& r_MB_B) { return r_MB_B.allFinite(); }
    static bool isValidR_FM_F(const Eigen::Vector3f& r_FM_F) { return r_FM_F.allFinite(); }
    static bool isValidThetaMax(float thetaMax) {
        return fsw::is_finite(thetaMax) && thetaMax > 0.0F && thetaMax < std::numbers::pi_v<float>;
    }

    const Eigen::Vector3f& getSigma_MB() const { return sigma_MB; }
    const Eigen::Vector3f& getR_MB_B() const { return r_MB_B; }
    const Eigen::Vector3f& getR_FM_F() const { return r_FM_F; }
    float getThetaMax() const { return thetaMax; }

   private:
    // NOLINTBEGIN(modernize-pass-by-value)
    // modernize-pass-by-value: this is a private constructor invoked only from create() with already-validated
    //   arguments; the small Eigen vectors are stored by copy without a move for clarity.
    ThrustVectoringConfig(const Eigen::Vector3f& sigma_MB,
                          const Eigen::Vector3f& r_MB_B,
                          const Eigen::Vector3f& r_FM_F,
                          float thetaMax)
        : sigma_MB(sigma_MB), r_MB_B(r_MB_B), r_FM_F(r_FM_F), thetaMax(thetaMax) {}
    // NOLINTEND(modernize-pass-by-value)

    Eigen::Vector3f sigma_MB;
    Eigen::Vector3f r_MB_B;
    Eigen::Vector3f r_FM_F;
    float thetaMax;
};

/*! @brief Pure algorithm computing the reference orientation of a thruster platform. */
class ThrustVectoringAlgorithm final {
   public:
    explicit ThrustVectoringAlgorithm(const ThrustVectoringConfig& config);
    void setConfig(const ThrustVectoringConfig& config);
    void reInitialize();
    ThrustVectoringOutput update(const ThrustVectoringInputs& in);

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
