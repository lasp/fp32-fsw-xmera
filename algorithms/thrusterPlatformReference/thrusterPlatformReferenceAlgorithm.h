#ifndef F32XMERA_THRUSTER_PLATFORM_REFERENCE_ALGORITHM_H
#define F32XMERA_THRUSTER_PLATFORM_REFERENCE_ALGORITHM_H

#include "thrusterPlatformReferenceTypes.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/freestandingIsFinite.hpp"
#include "utilities/fsw/rigidBodyKinematics.hpp"
#include <math.h>
#include <stdint.h>

#include <Eigen/Core>
#include <numbers>

/*! @brief Reaction-wheel spin-axis configuration used for momentum dumping. */
struct ThrusterPlatformReferenceRwArrayConfiguration {
    uint32_t numRW{};  //!< [-] number of reaction wheels on the vehicle
    Eigen::Matrix<float, 3, kMaxNumRw> GsMatrix_B{
        Eigen::Matrix<float, 3, kMaxNumRw>::Zero()};  //!< [-] RW spin axes in body frame, one column per wheel
    Eigen::Vector<float, kMaxNumRw> JsList{Eigen::Vector<float, kMaxNumRw>::Zero()};  //!< [kgm2] RW spin-axis inertias
};

/*! @brief Per-cycle inputs to the thruster platform reference algorithm. */
struct ThrusterPlatformReferenceInputs {
    Eigen::Vector3f r_CB_B{Eigen::Vector3f::Zero()};  //!< [m] center of mass w.r.t. B origin, B frame
    Eigen::Vector3f r_TF_F{Eigen::Vector3f::Zero()};  //!< [m] thrust application point w.r.t. F origin, F frame
    Eigen::Vector3f tHat_F{Eigen::Vector3f::Zero()};  //!< [-] thrust unit direction, F frame
    float thrust{};                                   //!< [N] thrust magnitude
    Eigen::Vector<float, kMaxNumRw> wheelSpeeds{
        Eigen::Vector<float, kMaxNumRw>::Zero()};  //!< [r/s] reaction-wheel speeds
};

/*! @brief Outputs of the thruster platform reference algorithm. */
struct ThrusterPlatformReferenceOutput {
    Eigen::Vector3f Lcomp_B{Eigen::Vector3f::Zero()};  //!< [Nm] torque to be compensated by the RWs, B frame
    Eigen::Vector3f r_TB_B{Eigen::Vector3f::Zero()};   //!< [m] thrust application point w.r.t. B origin, B frame
    Eigen::Vector3f tHat_B{Eigen::Vector3f::Zero()};   //!< [-] thrust unit direction, B frame
    float thrust{};                                    //!< [N] thrust magnitude
};

/*!
 * @brief Validated configuration for the thruster platform reference algorithm.
 *
 * Bundles the platform mounting geometry, the momentum-dumping gains, the thrust-deflection cone limit, and the
 * reaction-wheel configuration used for momentum dumping. An instance can only exist if the geometry vectors are
 * finite, the gains are finite and non-negative, the control period is finite and positive, the deflection cone
 * half-angle lies in (0, pi), and the reaction-wheel configuration count does not exceed the compile-time maximum
 * with finite spin axes and inertias. Construct via ThrusterPlatformReferenceConfig::create(...).
 */
class ThrusterPlatformReferenceConfig final {
   public:
    static ThrusterPlatformReferenceConfig create(const Eigen::Vector3f& sigma_MB,
                                                  const Eigen::Vector3f& r_BM_M,
                                                  const Eigen::Vector3f& r_FM_F,
                                                  float K,
                                                  float Ki,
                                                  float controlPeriod,
                                                  float thetaMax,
                                                  bool momentumDumping,
                                                  const ThrusterPlatformReferenceRwArrayConfiguration& rwConfig) {
        if (!isValidSigma_MB(sigma_MB)) {
            FSW_THROW_INVALID_ARGUMENT("thrusterPlatformReference: sigma_MB must be finite.");
        }
        if (!isValidR_BM_M(r_BM_M)) {
            FSW_THROW_INVALID_ARGUMENT("thrusterPlatformReference: r_BM_M must be finite.");
        }
        if (!isValidR_FM_F(r_FM_F)) {
            FSW_THROW_INVALID_ARGUMENT("thrusterPlatformReference: r_FM_F must be finite.");
        }
        if (!isValidK(K)) {
            FSW_THROW_INVALID_ARGUMENT("thrusterPlatformReference: K must be finite and non-negative.");
        }
        if (!isValidKi(Ki)) {
            FSW_THROW_INVALID_ARGUMENT("thrusterPlatformReference: Ki must be finite and non-negative.");
        }
        if (!isValidControlPeriod(controlPeriod)) {
            FSW_THROW_INVALID_ARGUMENT("thrusterPlatformReference: controlPeriod must be finite and positive.");
        }
        if (!isValidThetaMax(thetaMax)) {
            FSW_THROW_INVALID_ARGUMENT("thrusterPlatformReference: thetaMax must lie in the open interval (0, pi).");
        }
        if (!isValidRwConfig(rwConfig)) {
            FSW_THROW_INVALID_ARGUMENT(
                "thrusterPlatformReference: rwConfig.numRW must not exceed the compile-time maximum, its inertias "
                "must be finite, and each spin axis must be (close to) a unit vector.");
        }

        // Store unit-length spin axes so the momentum sum uses exact unit directions.
        ThrusterPlatformReferenceRwArrayConfiguration normalizedRwConfig = rwConfig;
        for (uint32_t i = 0U; i < normalizedRwConfig.numRW; ++i) {
            normalizedRwConfig.GsMatrix_B.col(i).normalize();
        }

        // Bound sigma_MB to the principal MRP set (norm <= 1) by switching to the shadow set if needed, so the
        // stored orientation is always a well-conditioned MRP representation.
        return {
            mrpSwitch(sigma_MB), r_BM_M, r_FM_F, K, Ki, controlPeriod, thetaMax, momentumDumping, normalizedRwConfig};
    }

    static bool isValidSigma_MB(const Eigen::Vector3f& sigma_MB) { return sigma_MB.allFinite(); }
    static bool isValidR_BM_M(const Eigen::Vector3f& r_BM_M) { return r_BM_M.allFinite(); }
    static bool isValidR_FM_F(const Eigen::Vector3f& r_FM_F) { return r_FM_F.allFinite(); }
    static bool isValidK(float K) { return fsw::is_finite(K) && K >= 0.0F; }
    static bool isValidKi(float Ki) { return fsw::is_finite(Ki) && Ki >= 0.0F; }
    static bool isValidControlPeriod(float controlPeriod) {
        return fsw::is_finite(controlPeriod) && controlPeriod > 0.0F;
    }
    static bool isValidThetaMax(float thetaMax) {
        return fsw::is_finite(thetaMax) && thetaMax > 0.0F && thetaMax < std::numbers::pi_v<float>;
    }
    static bool isValidRwConfig(const ThrusterPlatformReferenceRwArrayConfiguration& rwConfig) {
        if (rwConfig.numRW > static_cast<uint32_t>(kMaxNumRw) || !rwConfig.GsMatrix_B.allFinite() ||
            !rwConfig.JsList.allFinite()) {
            return false;
        }
        // Each spin axis must be (close to) a unit vector; they are normalized exactly on construction.
        constexpr float kUnitNormTol = 1e-3F;
        for (uint32_t i = 0U; i < rwConfig.numRW; ++i) {
            if (fabsf(rwConfig.GsMatrix_B.col(i).norm() - 1.0F) > kUnitNormTol) {
                return false;
            }
        }
        return true;
    }

    const Eigen::Vector3f& getSigma_MB() const { return sigma_MB; }
    const Eigen::Vector3f& getR_BM_M() const { return r_BM_M; }
    const Eigen::Vector3f& getR_FM_F() const { return r_FM_F; }
    float getK() const { return K; }
    float getKi() const { return Ki; }
    float getControlPeriod() const { return controlPeriod; }
    float getThetaMax() const { return thetaMax; }
    bool getMomentumDumping() const { return momentumDumping; }
    const ThrusterPlatformReferenceRwArrayConfiguration& getRwConfig() const { return rwConfig; }

   private:
    // NOLINTBEGIN(bugprone-easily-swappable-parameters, modernize-pass-by-value)
    // bugprone-easily-swappable-parameters: the geometry vectors and gains mirror the documented physical
    //   configuration; reordering them would be a caller error caught by the field-specific validators.
    // modernize-pass-by-value: this is a private constructor invoked only from create() with already-validated
    //   arguments; the small Eigen vectors are stored by copy without a move for clarity.
    ThrusterPlatformReferenceConfig(const Eigen::Vector3f& sigma_MB,
                                    const Eigen::Vector3f& r_BM_M,
                                    const Eigen::Vector3f& r_FM_F,
                                    float K,
                                    float Ki,
                                    float controlPeriod,
                                    float thetaMax,
                                    bool momentumDumping,
                                    const ThrusterPlatformReferenceRwArrayConfiguration& rwConfig)
        : sigma_MB(sigma_MB),
          r_BM_M(r_BM_M),
          r_FM_F(r_FM_F),
          K(K),
          Ki(Ki),
          controlPeriod(controlPeriod),
          thetaMax(thetaMax),
          momentumDumping(momentumDumping),
          rwConfig(rwConfig) {}
    // NOLINTEND(bugprone-easily-swappable-parameters, modernize-pass-by-value)

    Eigen::Vector3f sigma_MB;
    Eigen::Vector3f r_BM_M;
    Eigen::Vector3f r_FM_F;
    float K;
    float Ki;
    float controlPeriod;
    float thetaMax;
    bool momentumDumping;
    ThrusterPlatformReferenceRwArrayConfiguration rwConfig;
};

/*! @brief Pure algorithm computing the reference orientation of a thruster platform. */
class ThrusterPlatformReferenceAlgorithm final {
   public:
    explicit ThrusterPlatformReferenceAlgorithm(const ThrusterPlatformReferenceConfig& config);
    void setConfig(const ThrusterPlatformReferenceConfig& config);
    void reInitialize();
    ThrusterPlatformReferenceOutput update(const ThrusterPlatformReferenceInputs& in);

   private:
    /*! Advance the RW momentum integrator and return the momentum-dumping torque request in the platform frame. */
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) -- the vectors are distinct by frame and documented.
    Eigen::Vector3f computeDumpingTorque(const ThrusterPlatformReferenceInputs& in,
                                         const Eigen::Vector3f& r_CM_M,
                                         const Eigen::Vector3f& r_TM_F,
                                         const Eigen::Vector3f& thrust_F,
                                         const Eigen::Matrix3f& dcm_MB);

    ThrusterPlatformReferenceConfig cfg;                 //!< [-] validated configuration
    Eigen::Vector3f hsInt_B{Eigen::Vector3f::Zero()};    //!< [Nms] integral of RW momentum, B frame
    Eigen::Vector3f priorHs_B{Eigen::Vector3f::Zero()};  //!< [Nms] prior RW momentum, B frame
    Eigen::Matrix3f priorDcm_FM{
        Eigen::Matrix3f::Zero()};  //!< [-] previous cycle's reference DCM [FM] (torque-conversion seed; zero if unset)
};

#endif  // F32XMERA_THRUSTER_PLATFORM_REFERENCE_ALGORITHM_H
