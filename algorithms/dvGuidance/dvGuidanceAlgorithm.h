#ifndef F32XMERA_DV_GUIDANCE_ALGORITHM_H
#define F32XMERA_DV_GUIDANCE_ALGORITHM_H

#include "utilities/fsw/freestandingInvalidArgument.h"
#include <math.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

/// Burn-frame attitude guidance output. The default (all-zero) value represents
/// the identity R/N attitude with zero rates.
struct DvGuidanceOutput {
    Eigen::Vector3f sigma_RN = Eigen::Vector3f::Zero();     ///< MRP of the reference frame w.r.t. inertial.
    Eigen::Vector3f omega_RN_N = Eigen::Vector3f::Zero();   ///< Reference angular rate, inertial frame [rad/s].
    Eigen::Vector3f domega_RN_N = Eigen::Vector3f::Zero();  ///< Reference angular acceleration, inertial frame.
};

/// Validated, immutable configuration for the delta-V burn reference frame. Construct via create(),
/// which enforces the parameter constraints and throws fsw::invalid_argument on a violation.
class DvGuidanceConfig final {
   public:
    /// Delta-V norm floor (1e-6 m/s); below this the burn direction is undefined.
    static constexpr float kMinNorm = 1e-6F;
    /// Cross-product norm floor (~0.06 deg from (anti)parallel); below this the burn frame is degenerate.
    static constexpr float kMinCross = 1e-3F;

    // cmdDv_N and rHatPref_N share the Eigen::Vector3f type but have distinct roles; construction is
    // funneled through the named create() factory, which makes the argument roles explicit at every call site.
    // NOLINTBEGIN(bugprone-easily-swappable-parameters)

    /// @param cmdDv_N    [m/s] Commanded delta-V vector, inertial frame; defines the burn frame's 3rd axis
    /// @param rHatPref_N [-] Preferred unit direction of the burn frame's 2nd axis, inertial frame; only its
    ///                   component perpendicular to cmdDv_N is used
    static DvGuidanceConfig create(const Eigen::Vector3f& cmdDv_N, const Eigen::Vector3f& rHatPref_N) {
        if (!isValidCmdDv(cmdDv_N)) {
            FSW_THROW_INVALID_ARGUMENT("dvGuidance: cmdDv_N must be finite and nonzero.");
        }
        if (!isValidRHatPref(cmdDv_N, rHatPref_N)) {
            FSW_THROW_INVALID_ARGUMENT(
                "dvGuidance: rHatPref_N must be finite, within 1e-3 of unit length, and not parallel to cmdDv_N.");
        }
        return {cmdDv_N, rHatPref_N.stableNormalized()};
    }
    static bool isValidCmdDv(const Eigen::Vector3f& cmdDv_N) {
        return cmdDv_N.allFinite() && cmdDv_N.stableNorm() >= kMinNorm;
    }
    static bool isValidRHatPref(const Eigen::Vector3f& cmdDv_N, const Eigen::Vector3f& rHatPref_N) {
        constexpr float kNormTolerance = 1e-3F;
        return rHatPref_N.allFinite() && fabsf(rHatPref_N.stableNorm() - 1.0F) < kNormTolerance &&
               rHatPref_N.stableNormalized().cross(cmdDv_N.stableNormalized()).stableNorm() >= kMinCross;
    }

    const Eigen::Vector3f& getCmdDv() const { return cmdDv_N; }
    const Eigen::Vector3f& getRHatPref() const { return rHatPref_N; }

   private:
    DvGuidanceConfig(const Eigen::Vector3f& cmdDv_N, const Eigen::Vector3f& rHatPref_N)
        : cmdDv_N(cmdDv_N), rHatPref_N(rHatPref_N) {}
    // NOLINTEND(bugprone-easily-swappable-parameters)

    Eigen::Vector3f cmdDv_N{Eigen::Vector3f::Zero()};     ///< [m/s] commanded delta-V, inertial frame
    Eigen::Vector3f rHatPref_N{Eigen::Vector3f::Zero()};  ///< [-] preferred 2nd-axis direction, inertial frame (unit)
};

/// Computes the inertially fixed delta-V burn-frame attitude reference from a commanded delta-V
/// and a rotation-axis seed. All math is single-precision (FP32). Degenerate burn-frame geometry
/// returns the default output.
class DvGuidanceAlgorithm final {
   public:
    /// Squared delta-V norm threshold (~1e-6 m/s); below this the burn direction is treated as undefined.
    static constexpr float kMinNormSq = 1e-12F;
    /// Squared cross-product threshold (~1.7 deg from (anti)parallel); below this the base
    /// burn-frame geometry is treated as degenerate.
    static constexpr float kMinCrossSq = 9e-4F;

    /// Computes the burn-frame attitude reference.
    /// @param dvInrtlCmd    Commanded delta-V in inertial frame [m/s]; defines the 1st burn-frame axis
    /// @param dvRotVecUnit  Rotation-axis seed; only its direction is used (need not be unit)
    /// @return The burn-frame attitude reference with zero rates, or the default output if the burn
    ///         frame cannot be constructed.
    static DvGuidanceOutput update(const Eigen::Vector3f& dvInrtlCmd, const Eigen::Vector3f& dvRotVecUnit);
};

#endif
