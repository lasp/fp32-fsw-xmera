#ifndef F32XMERA_DV_GUIDANCE_ALGORITHM_H
#define F32XMERA_DV_GUIDANCE_ALGORITHM_H

#include "dvGuidanceTypes.h"
#include <stdint.h>
#include <Eigen/Core>

/// Burn-frame attitude guidance output. On degenerate input the algorithm returns the
/// default (all-zero) value: @c sigma_RN = 0 is the identity R/N attitude with zero rates.
struct DvGuidanceOutput {
    Eigen::Vector3f sigma_RN = Eigen::Vector3f::Zero();     ///< MRP of the reference frame w.r.t. inertial.
    Eigen::Vector3f omega_RN_N = Eigen::Vector3f::Zero();   ///< Reference angular rate, inertial frame [rad/s].
    Eigen::Vector3f domega_RN_N = Eigen::Vector3f::Zero();  ///< Reference angular acceleration, inertial frame.
};

/// Configuration for DvGuidanceAlgorithm. dvGuidance has no tunable parameters; this class is
/// intentionally empty so the algorithm still follows the standard two-phase init pattern.
class DvGuidanceConfig final {
   public:
    static DvGuidanceConfig create() { return {}; }

   private:
    DvGuidanceConfig() = default;
};

/// Computes the delta-V burn-frame attitude reference from a commanded delta-V, a rotation axis,
/// and a rotation rate. All math is single-precision (FP32). Degenerate inputs are guarded so the
/// module never propagates NaN or numerical noise downstream; see dvGuidance.rst for the rationale
/// behind the threshold constants below.
class DvGuidanceAlgorithm final {
   public:
    /// @c |dvInrtlCmd|^2 floor (~1e-6 m/s): at or below this the delta-V direction is undefined.
    static constexpr float kMinNormSq = 1e-12F;
    /// @c |rHat x dvHat|^2 = sin^2(angle) floor (sin ~ 3e-2, ~1.7 deg): below this the base burn
    /// frame is ill-defined / FP32-noise-dominated, so the rotation axis is treated as (anti)parallel.
    static constexpr float kMinCrossSq = 9e-4F;
    /// @c |dvRotVecMag * burnTime| (rad) floor: smaller rotations are reported as identity rather
    /// than the FP32 noise that would otherwise dominate the sub-threshold deviation.
    static constexpr float kSmallAngle = 1e-5F;

    explicit DvGuidanceAlgorithm(const DvGuidanceConfig& config);

    void setConfig(const DvGuidanceConfig& config);

    /// Computes the burn-frame attitude reference.
    /// @param dvInrtlCmd    Commanded delta-V in inertial frame [m/s]; defines the 1st burn-frame axis.
    /// @param dvRotVecUnit  Rotation axis seed; only its direction is used (need not be unit).
    /// @param dvRotVecMag   Burn-frame rotation rate about the 3rd axis [rad/s].
    /// @param burnStartTime Burn epoch [ns].
    /// @param callTime      Evaluation time [ns]; (callTime - burnStartTime) is the elapsed burn time.
    /// @return The attitude reference, or a default (identity, zero-rate) DvGuidanceOutput when
    ///         @p dvInrtlCmd is near-zero or @p dvRotVecUnit is (anti)parallel to it.
    DvGuidanceOutput update(const Eigen::Vector3f& dvInrtlCmd,
                            const Eigen::Vector3f& dvRotVecUnit,
                            float dvRotVecMag,
                            uint64_t burnStartTime,
                            uint64_t callTime) const;

   private:
    DvGuidanceConfig cfg;
};

#endif
