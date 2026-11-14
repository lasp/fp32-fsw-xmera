#ifndef F32XMERA_DV_GUIDANCE_ALGORITHM_H
#define F32XMERA_DV_GUIDANCE_ALGORITHM_H

#include "utilities/fsw/freestandingInvalidArgument.h"
#include <math.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

/// Validated, immutable configuration for the delta-V burn reference frame. Construct via create(),
/// which enforces the parameter constraints and throws fsw::invalid_argument on a violation.
class DvGuidanceConfig final {
   public:
    /// Delta-V norm floor (1e-6 m/s); below this the burn direction is undefined.
    static constexpr float kMinNorm = 1e-6F;
    /// Cross-product norm floor (~0.06 deg from (anti)parallel); below this the burn frame is degenerate.
    static constexpr float kMinCross = 1e-3F;

    // requestedDv_N and rHatPref_N share the Eigen::Vector3f type but have distinct roles; construction is
    // funneled through the named create() factory, which makes the argument roles explicit at every call site.
    // NOLINTBEGIN(bugprone-easily-swappable-parameters)

    /// @param requestedDv_N [m/s] Requested delta-V vector, inertial frame; defines the burn frame's 3rd axis
    /// @param rHatPref_N    [-] Preferred unit direction of the burn frame's 2nd axis, inertial frame; only its
    ///                      component perpendicular to requestedDv_N is used
    static DvGuidanceConfig create(const Eigen::Vector3f& requestedDv_N, const Eigen::Vector3f& rHatPref_N) {
        if (!isValidRequestedDv(requestedDv_N)) {
            FSW_THROW_INVALID_ARGUMENT("dvGuidance: requestedDv_N must be finite and nonzero.");
        }
        if (!isValidRHatPref(requestedDv_N, rHatPref_N)) {
            FSW_THROW_INVALID_ARGUMENT(
                "dvGuidance: rHatPref_N must be finite, within 1e-3 of unit length, and not parallel to "
                "requestedDv_N.");
        }
        return {requestedDv_N, rHatPref_N.stableNormalized()};
    }
    static bool isValidRequestedDv(const Eigen::Vector3f& requestedDv_N) {
        return requestedDv_N.allFinite() && requestedDv_N.stableNorm() >= kMinNorm;
    }
    static bool isValidRHatPref(const Eigen::Vector3f& requestedDv_N, const Eigen::Vector3f& rHatPref_N) {
        constexpr float kNormTolerance = 1e-3F;
        return rHatPref_N.allFinite() && fabsf(rHatPref_N.stableNorm() - 1.0F) < kNormTolerance &&
               rHatPref_N.stableNormalized().cross(requestedDv_N.stableNormalized()).stableNorm() >= kMinCross;
    }

    const Eigen::Vector3f& getRequestedDv() const { return requestedDv_N; }
    const Eigen::Vector3f& getRHatPref() const { return rHatPref_N; }

   private:
    DvGuidanceConfig(const Eigen::Vector3f& requestedDv_N, const Eigen::Vector3f& rHatPref_N)
        : requestedDv_N(requestedDv_N), rHatPref_N(rHatPref_N) {}
    // NOLINTEND(bugprone-easily-swappable-parameters)

    Eigen::Vector3f requestedDv_N{Eigen::Vector3f::Zero()};  ///< [m/s] requested delta-V, inertial frame
    Eigen::Vector3f rHatPref_N{Eigen::Vector3f::Zero()};  ///< [-] preferred 2nd-axis direction, inertial frame (unit)
};

/// Computes the inertially fixed delta-V burn-frame attitude reference from the configured
/// requested delta-V and preferred 2nd-axis direction. All math is single-precision (FP32).
class DvGuidanceAlgorithm final {
   public:
    /// Constructs the algorithm from a validated configuration.
    explicit DvGuidanceAlgorithm(const DvGuidanceConfig& config);

    /// Installs the configuration parameters and computes the inertially fixed burn frame.
    void setConfig(const DvGuidanceConfig& config);

    /// @return [-] The fixed burn-frame MRP sigma_RN computed by setConfig(); the reference rates are zero.
    Eigen::Vector3f update() const;

   private:
    DvGuidanceConfig cfg;                               ///< validated configuration
    Eigen::Vector3f sigma_RN{Eigen::Vector3f::Zero()};  ///< [-] MRP of the burn frame R relative to N
};

#endif
