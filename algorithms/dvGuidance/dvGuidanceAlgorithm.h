#ifndef F32XMERA_DV_GUIDANCE_ALGORITHM_H
#define F32XMERA_DV_GUIDANCE_ALGORITHM_H

#include <stdint.h>
#include <Eigen/Core>

/// Burn-frame attitude guidance output. The default (all-zero) value represents
/// the identity R/N attitude with zero rates.
struct DvGuidanceOutput {
    Eigen::Vector3f sigma_RN = Eigen::Vector3f::Zero();     ///< MRP of the reference frame w.r.t. inertial.
    Eigen::Vector3f omega_RN_N = Eigen::Vector3f::Zero();   ///< Reference angular rate, inertial frame [rad/s].
    Eigen::Vector3f domega_RN_N = Eigen::Vector3f::Zero();  ///< Reference angular acceleration, inertial frame.
};

/// Computes the time-varying delta-V burn-frame attitude reference from a commanded delta-V,
/// a rotation-axis seed, a constant rotation rate, and the elapsed time relative to the burn start.
/// All math is single-precision (FP32). Degenerate burn-frame geometry returns the default output.
class DvGuidanceAlgorithm final {
   public:
    /// Squared delta-V norm threshold (~1e-6 m/s); below this the burn direction is treated as undefined.
    static constexpr float kMinNormSq = 1e-12F;
    /// Squared cross-product threshold (~1.7 deg from (anti)parallel); below this the base
    /// burn-frame geometry is treated as degenerate.
    static constexpr float kMinCrossSq = 9e-4F;
    /// Burn-frame rotation-angle threshold (~0.00057 deg); smaller rotations from the base
    /// burn frame are treated as zero.
    static constexpr float kSmallAngle = 1e-5F;

    /// Computes the burn-frame attitude reference.
    /// @param dvInrtlCmd    Commanded delta-V in inertial frame [m/s]; defines the 1st burn-frame axis
    /// @param dvRotVecUnit  Rotation-axis seed; only its direction is used (need not be unit)
    /// @param dvRotVecMag   Constant burn-frame rotation rate about the 3rd axis [rad/s]
    /// @param burnStartTime Burn start time [ns]
    /// @param callTime      Evaluation time [ns]
    /// @return The calculated attitude reference, or the default output if the burn frame
    ///         cannot be constructed.
    static DvGuidanceOutput update(const Eigen::Vector3f& dvInrtlCmd,
                                   const Eigen::Vector3f& dvRotVecUnit,
                                   float dvRotVecMag,
                                   uint64_t burnStartTime,
                                   uint64_t callTime);
};

#endif
