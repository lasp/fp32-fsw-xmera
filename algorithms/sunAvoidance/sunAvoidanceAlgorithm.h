#ifndef F32XMERA_SUN_AVOIDANCE_ALGORITHM_H
#define F32XMERA_SUN_AVOIDANCE_ALGORITHM_H

#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/freestandingIsFinite.hpp"
#include <math.h>
#include <stdint.h>
#include <Eigen/Core>
#include <optional>

struct SunAvoidanceAttRefInputs {
    Eigen::Vector3f sigma_RN{Eigen::Vector3f::Zero()};     //!< [-] reference MRP attitude of R wrt inertial N
    Eigen::Vector3f omega_RN_N{Eigen::Vector3f::Zero()};   //!< [r/s] reference rate of R wrt N in N frame
    Eigen::Vector3f domega_RN_N{Eigen::Vector3f::Zero()};  //!< [r/s^2] reference angular acceleration in N frame
};

// The maneuver-adjusted reference frame: the input reference rotated by the Sun-avoidance maneuver.
struct SunAvoidanceOutput {
    Eigen::Vector3f sigma_RN{Eigen::Vector3f::Zero()};     //!< [-] adjusted reference MRP wrt inertial N
    Eigen::Vector3f omega_RN_N{Eigen::Vector3f::Zero()};   //!< [r/s] adjusted reference rate, N-frame components
    Eigen::Vector3f domega_RN_N{Eigen::Vector3f::Zero()};  //!< [r/s^2] adjusted reference angular acceleration, N frame
};

class SunAvoidanceConfig final {
   public:
    static SunAvoidanceConfig create(const Eigen::Vector3f& sensitiveHat_B, float angleRate, bool computeAngleStart) {
        // sensitiveHat_B is only used when the Sun-avoidance maneuver is enabled; validate it only then.
        if (computeAngleStart && !isValidSensitiveHat_B(sensitiveHat_B)) {
            FSW_THROW_INVALID_ARGUMENT("sunAvoidance: sensitiveHat_B must be finite and within 1e-3 of unit length");
        }
        if (!isValidAngleRate(angleRate)) {
            FSW_THROW_INVALID_ARGUMENT("sunAvoidance: angleRate must be finite");
        }
        return {sensitiveHat_B.normalized(), angleRate, computeAngleStart};
    }

    static bool isValidSensitiveHat_B(const Eigen::Vector3f& sensitiveHat_B) {
        constexpr float kNormTolerance = 1e-3F;
        return sensitiveHat_B.allFinite() && fabsf(sensitiveHat_B.stableNorm() - 1.0F) < kNormTolerance;
    }
    static bool isValidAngleRate(float angleRate) { return fsw::is_finite(angleRate); }

    Eigen::Vector3f getSensitiveHat_B() const { return sensitiveHat_B; }
    float getAngleRate() const { return angleRate; }
    bool getComputeAngleStart() const { return computeAngleStart; }

   private:
    SunAvoidanceConfig(const Eigen::Vector3f& sensitiveHat_B, float angleRate, bool computeAngleStart)
        : sensitiveHat_B(sensitiveHat_B), angleRate(angleRate), computeAngleStart(computeAngleStart) {}

    Eigen::Vector3f sensitiveHat_B;
    float angleRate;
    bool computeAngleStart;
};

/*!@brief Module to compute the attitude tracking error for sun avoidance.
 */
class SunAvoidanceAlgorithm final {
   public:
    explicit SunAvoidanceAlgorithm(const SunAvoidanceConfig& config);

    void setConfig(const SunAvoidanceConfig& config);

    void reInitialize();
    SunAvoidanceOutput update(const Eigen::Vector3f& sigma_BN,
                              const SunAvoidanceAttRefInputs& ref,
                              const Eigen::Vector3d& r_BN_N,
                              const Eigen::Vector3d& r_SN_N,
                              uint64_t callTime);

   private:
    struct Maneuver {
        Eigen::Vector3f axis_B{Eigen::Vector3f::Zero()};  //!< [-] principal rotation axis of the maneuver
        float angle{};                                    //!< [rad] total maneuver rotation angle
        uint64_t startTime{};                             //!< [ns] time at which the maneuver began
    };

    Maneuver initializeManeuver(const Eigen::Vector3f& sigma_BN,
                                const SunAvoidanceAttRefInputs& ref,
                                const Eigen::Vector3f& sHat_N) const;
    SunAvoidanceOutput computeAdjustedReference(const Eigen::Vector3f& sigma_BN,
                                                const SunAvoidanceAttRefInputs& ref,
                                                uint64_t callTime) const;

    SunAvoidanceConfig cfg;

    std::optional<Maneuver> maneuver;  //!< runtime maneuver state; empty until initialized
};

#endif
