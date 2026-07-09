#ifndef F32XMERA_SUN_TRACK_ERROR_ALGORITHM_H
#define F32XMERA_SUN_TRACK_ERROR_ALGORITHM_H

#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/freestandingIsFinite.hpp"
#include <math.h>
#include <stdint.h>
#include <Eigen/Core>

struct SunTrackErrorAttRefInputs {
    Eigen::Vector3f sigma_RN{Eigen::Vector3f::Zero()};     //!< [-] reference MRP attitude of R wrt inertial N
    Eigen::Vector3f omega_RN_N{Eigen::Vector3f::Zero()};   //!< [r/s] reference rate of R wrt N in N frame
    Eigen::Vector3f domega_RN_N{Eigen::Vector3f::Zero()};  //!< [r/s^2] reference angular acceleration in N frame
};

// The maneuver-adjusted reference frame: the input reference rotated by the Sun-avoidance maneuver.
struct SunTrackErrorOutput {
    Eigen::Vector3f sigma_RN{Eigen::Vector3f::Zero()};     //!< [-] adjusted reference MRP wrt inertial N
    Eigen::Vector3f omega_RN_N{Eigen::Vector3f::Zero()};   //!< [r/s] adjusted reference rate, N-frame components
    Eigen::Vector3f domega_RN_N{Eigen::Vector3f::Zero()};  //!< [r/s^2] adjusted reference angular acceleration, N frame
};

class SunTrackErrorConfig final {
   public:
    static SunTrackErrorConfig create(const Eigen::Vector3f& sensitiveHat_B, float angleRate, bool computeAngleStart) {
        // sensitiveHat_B is only used when the Sun-avoidance maneuver is enabled; validate it only then.
        if (computeAngleStart && !isValidSensitiveHat_B(sensitiveHat_B)) {
            FSW_THROW_INVALID_ARGUMENT("sunTrackError: sensitiveHat_B must be finite and within 1e-3 of unit length");
        }
        if (!isValidAngleRate(angleRate)) {
            FSW_THROW_INVALID_ARGUMENT("sunTrackError: angleRate must be finite");
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
    SunTrackErrorConfig(const Eigen::Vector3f& sensitiveHat_B, float angleRate, bool computeAngleStart)
        : sensitiveHat_B(sensitiveHat_B), angleRate(angleRate), computeAngleStart(computeAngleStart) {}

    Eigen::Vector3f sensitiveHat_B;
    float angleRate;
    bool computeAngleStart;
};

/*!@brief Module to compute the attitude tracking error for sun avoidance.
 */
class SunTrackErrorAlgorithm final {
   public:
    explicit SunTrackErrorAlgorithm(const SunTrackErrorConfig& config);

    void setConfig(const SunTrackErrorConfig& config);

    void reInitialize();
    SunTrackErrorOutput update(const Eigen::Vector3f& sigma_BN,
                               const SunTrackErrorAttRefInputs& ref,
                               const Eigen::Vector3f& r_BN_N,
                               const Eigen::Vector3f& r_SN_N,
                               uint64_t callTime);

   private:
    void initializeManeuver(const Eigen::Vector3f& sigma_BN,
                            const SunTrackErrorAttRefInputs& ref,
                            const Eigen::Vector3f& sHat_N);
    SunTrackErrorOutput computeAdjustedReference(const Eigen::Vector3f& sigma_BN,
                                                 const SunTrackErrorAttRefInputs& ref,
                                                 uint64_t callTime) const;

    SunTrackErrorConfig cfg;

    Eigen::Vector3f maneuverAxis_B{Eigen::Vector3f::Zero()};  //!< [-] principal rotation axis of the maneuver
    float maneuverAngle{};         //!< [rad] total maneuver rotation angle, set at initialization
    bool maneuverInitialized{};    //!< [-] whether the maneuver has been initialized
    uint64_t maneuverStartTime{};  //!< [ns] time at which the maneuver began
};

#endif
