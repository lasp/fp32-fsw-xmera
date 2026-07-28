#ifndef F32XMERA_THRUSTER_PLATFORM_REFERENCE_ALGORITHM_H
#define F32XMERA_THRUSTER_PLATFORM_REFERENCE_ALGORITHM_H

#include "thrusterPlatformReferenceTypes.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/freestandingIsFinite.hpp"
#include <math.h>
#include <stdint.h>

#include <Eigen/Core>

inline constexpr int kMaxNumRw = THRUSTER_PLATFORM_REFERENCE_MAX_NUM_RW;  //!< [-] maximum number of reaction wheels

/*! @brief Reaction-wheel spin-axis configuration used for momentum dumping. */
struct ThrusterPlatformReferenceRwArrayConfig {
    uint32_t numRW{};  //!< [-] number of reaction wheels on the vehicle
    Eigen::Matrix<float, 3, kMaxNumRw> GsMatrix_B{
        Eigen::Matrix<float, 3, kMaxNumRw>::Zero()};  //!< [-] RW spin axes in body frame, one column per wheel
    Eigen::Vector<float, kMaxNumRw> JsList{Eigen::Vector<float, kMaxNumRw>::Zero()};  //!< [kgm2] RW spin-axis inertias
};

/*! @brief Per-cycle inputs to the thruster platform reference algorithm. */
struct ThrusterPlatformReferenceInputs {
    Eigen::Vector3f r_CB_B{Eigen::Vector3f::Zero()};        //!< [m] center of mass w.r.t. B origin, B frame
    Eigen::Vector3f rThrust_F{Eigen::Vector3f::Zero()};     //!< [m] thrust application point w.r.t. F origin, F frame
    Eigen::Vector3f tHatThrust_F{Eigen::Vector3f::Zero()};  //!< [-] thrust unit direction, F frame
    float maxThrust{};                                      //!< [N] thrust magnitude
    Eigen::Vector<float, kMaxNumRw> wheelSpeeds{
        Eigen::Vector<float, kMaxNumRw>::Zero()};  //!< [r/s] reaction-wheel speeds
};

/*! @brief Outputs of the thruster platform reference algorithm. */
struct ThrusterPlatformReferenceOutput {
    float theta1{};                                              //!< [rad] platform tip reference angle
    float theta2{};                                              //!< [rad] platform tilt reference angle
    Eigen::Vector3f torqueRequestBody{Eigen::Vector3f::Zero()};  //!< [Nm] torque to be compensated by the RWs, B frame
    Eigen::Vector3f rThrust_B{Eigen::Vector3f::Zero()};     //!< [m] thrust application point w.r.t. B origin, B frame
    Eigen::Vector3f tHatThrust_B{Eigen::Vector3f::Zero()};  //!< [-] thrust unit direction, B frame
    float maxThrust{};                                      //!< [N] thrust magnitude
};

/*!
 * @brief Validated configuration for the thruster platform reference algorithm.
 *
 * Bundles the platform mounting geometry, the momentum-dumping gains, the tip/tilt angle bounds, and the
 * reaction-wheel configuration used for momentum dumping. An instance can only exist if the geometry vectors are
 * finite, the gains are finite and non-negative, the angle bounds are finite (a non-positive bound disables
 * clamping on that axis), and the reaction-wheel configuration count does not exceed the compile-time maximum with
 * finite spin axes and inertias. Construct via ThrusterPlatformReferenceConfig::create(...).
 */
class ThrusterPlatformReferenceConfig final {
   public:
    static ThrusterPlatformReferenceConfig create(const Eigen::Vector3f& sigma_MB,
                                                  const Eigen::Vector3f& r_BM_M,
                                                  const Eigen::Vector3f& r_FM_F,
                                                  float K,
                                                  float Ki,
                                                  float theta1Max,
                                                  float theta2Max,
                                                  bool momentumDumping,
                                                  const ThrusterPlatformReferenceRwArrayConfig& rwConfig) {
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
        if (!isValidTheta1Max(theta1Max)) {
            FSW_THROW_INVALID_ARGUMENT("thrusterPlatformReference: theta1Max must be finite.");
        }
        if (!isValidTheta2Max(theta2Max)) {
            FSW_THROW_INVALID_ARGUMENT("thrusterPlatformReference: theta2Max must be finite.");
        }
        if (!isValidRwConfig(rwConfig)) {
            FSW_THROW_INVALID_ARGUMENT(
                "thrusterPlatformReference: rwConfig.numRW must not exceed the compile-time maximum, its inertias "
                "must be finite, and each spin axis must be (close to) a unit vector.");
        }

        // Store unit-length spin axes so the momentum sum uses exact unit directions.
        ThrusterPlatformReferenceRwArrayConfig normalizedRwConfig = rwConfig;
        for (uint32_t i = 0U; i < normalizedRwConfig.numRW; ++i) {
            normalizedRwConfig.GsMatrix_B.col(i).normalize();
        }

        return {sigma_MB, r_BM_M, r_FM_F, K, Ki, theta1Max, theta2Max, momentumDumping, normalizedRwConfig};
    }

    static bool isValidSigma_MB(const Eigen::Vector3f& sigma_MB) { return sigma_MB.allFinite(); }
    static bool isValidR_BM_M(const Eigen::Vector3f& r_BM_M) { return r_BM_M.allFinite(); }
    static bool isValidR_FM_F(const Eigen::Vector3f& r_FM_F) { return r_FM_F.allFinite(); }
    static bool isValidK(float K) { return fsw::is_finite(K) && K >= 0.0F; }
    static bool isValidKi(float Ki) { return fsw::is_finite(Ki) && Ki >= 0.0F; }
    static bool isValidTheta1Max(float theta1Max) { return fsw::is_finite(theta1Max); }
    static bool isValidTheta2Max(float theta2Max) { return fsw::is_finite(theta2Max); }
    static bool isValidRwConfig(const ThrusterPlatformReferenceRwArrayConfig& rwConfig) {
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
    float getTheta1Max() const { return theta1Max; }
    float getTheta2Max() const { return theta2Max; }
    bool getMomentumDumping() const { return momentumDumping; }
    const ThrusterPlatformReferenceRwArrayConfig& getRwConfig() const { return rwConfig; }

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
                                    float theta1Max,
                                    float theta2Max,
                                    bool momentumDumping,
                                    const ThrusterPlatformReferenceRwArrayConfig& rwConfig)
        : sigma_MB(sigma_MB),
          r_BM_M(r_BM_M),
          r_FM_F(r_FM_F),
          K(K),
          Ki(Ki),
          theta1Max(theta1Max),
          theta2Max(theta2Max),
          momentumDumping(momentumDumping),
          rwConfig(rwConfig) {}
    // NOLINTEND(bugprone-easily-swappable-parameters, modernize-pass-by-value)

    Eigen::Vector3f sigma_MB;
    Eigen::Vector3f r_BM_M;
    Eigen::Vector3f r_FM_F;
    float K;
    float Ki;
    float theta1Max;
    float theta2Max;
    bool momentumDumping;
    ThrusterPlatformReferenceRwArrayConfig rwConfig;
};

/*! @brief Pure algorithm computing the tip/tilt reference of a dual-gimballed thruster platform. */
class ThrusterPlatformReferenceAlgorithm final {
   public:
    explicit ThrusterPlatformReferenceAlgorithm(const ThrusterPlatformReferenceConfig& config);
    void setConfig(const ThrusterPlatformReferenceConfig& config);
    void reInitialize();
    ThrusterPlatformReferenceOutput update(const ThrusterPlatformReferenceInputs& in, uint64_t callTime);

   private:
    ThrusterPlatformReferenceConfig cfg;                 //!< [-] validated configuration
    Eigen::Vector3f hsInt_M{Eigen::Vector3f::Zero()};    //!< [Nms] integral of RW momentum, M frame
    Eigen::Vector3f priorHs_M{Eigen::Vector3f::Zero()};  //!< [Nms] prior RW momentum, M frame
    uint64_t priorTime{};                                //!< [ns] prior call time
};

#endif  // F32XMERA_THRUSTER_PLATFORM_REFERENCE_ALGORITHM_H
