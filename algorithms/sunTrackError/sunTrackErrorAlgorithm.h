#ifndef F32XMERA_SUN_TRACK_ERROR_ALGORITHM_H
#define F32XMERA_SUN_TRACK_ERROR_ALGORITHM_H

#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/freestandingIsFinite.hpp"
#include <math.h>
#include <stdint.h>
#include <Eigen/Core>

struct SunTrackErrorNavAttInputs {
    Eigen::Vector3f sigma_BN{Eigen::Vector3f::Zero()};    //!< [-] measured MRP attitude of B wrt inertial N
    Eigen::Vector3f omega_BN_B{Eigen::Vector3f::Zero()};  //!< [r/s] measured body rate of B wrt N in B frame
};

struct SunTrackErrorAttRefInputs {
    Eigen::Vector3f sigma_RN{Eigen::Vector3f::Zero()};     //!< [-] reference MRP attitude of R wrt inertial N
    Eigen::Vector3f omega_RN_N{Eigen::Vector3f::Zero()};   //!< [r/s] reference rate of R wrt N in N frame
    Eigen::Vector3f domega_RN_N{Eigen::Vector3f::Zero()};  //!< [r/s^2] reference angular acceleration in N frame
};

struct SunTrackErrorOutput {
    Eigen::Vector3f sigma_BR{Eigen::Vector3f::Zero()};     //!< [-] attitude error MRP of B wrt R
    Eigen::Vector3f omega_BR_B{Eigen::Vector3f::Zero()};   //!< [r/s] body rate error of B wrt R in B frame
    Eigen::Vector3f omega_RN_B{Eigen::Vector3f::Zero()};   //!< [r/s] reference rate of R wrt N in B frame
    Eigen::Vector3f domega_RN_B{Eigen::Vector3f::Zero()};  //!< [r/s^2] reference angular acceleration in B frame
};

class SunTrackErrorConfig final {
   public:
    static SunTrackErrorConfig create(const Eigen::Vector3f& sigma_R0R,
                                      const Eigen::Vector3f& sensitiveHat_B,
                                      float angleRate,
                                      bool computeAngleStart) {
        if (!isValidSigma_R0R(sigma_R0R)) {
            FSW_THROW_INVALID_ARGUMENT("sunTrackError: sigma_R0R must be finite");
        }
        // sensitiveHat_B is only used when the Sun-avoidance maneuver is enabled; validate it only then.
        if (computeAngleStart && !isValidSensitiveHat_B(sensitiveHat_B)) {
            FSW_THROW_INVALID_ARGUMENT("sunTrackError: sensitiveHat_B must be finite and within 1e-3 of unit length");
        }
        if (!isValidAngleRate(angleRate)) {
            FSW_THROW_INVALID_ARGUMENT("sunTrackError: angleRate must be finite");
        }
        return {sigma_R0R, sensitiveHat_B.normalized(), angleRate, computeAngleStart};
    }

    static bool isValidSigma_R0R(const Eigen::Vector3f& sigma) { return sigma.allFinite(); }
    static bool isValidSensitiveHat_B(const Eigen::Vector3f& sensitiveHat_B) {
        constexpr float kNormTolerance = 1e-3F;
        return sensitiveHat_B.allFinite() && fabsf(sensitiveHat_B.stableNorm() - 1.0F) < kNormTolerance;
    }
    static bool isValidAngleRate(float angleRate) { return fsw::is_finite(angleRate); }

    Eigen::Vector3f getSigma_R0R() const { return sigma_R0R; }
    Eigen::Vector3f getSensitiveHat_B() const { return sensitiveHat_B; }
    float getAngleRate() const { return angleRate; }
    bool getComputeAngleStart() const { return computeAngleStart; }

   private:
    SunTrackErrorConfig(const Eigen::Vector3f& sigma_R0R,
                        const Eigen::Vector3f& sensitiveHat_B,
                        float angleRate,
                        bool computeAngleStart)
        : sigma_R0R(sigma_R0R),
          sensitiveHat_B(sensitiveHat_B),
          angleRate(angleRate),
          computeAngleStart(computeAngleStart) {}

    Eigen::Vector3f sigma_R0R;
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
    SunTrackErrorOutput update(const SunTrackErrorNavAttInputs& nav,
                               const SunTrackErrorAttRefInputs& ref,
                               const Eigen::Vector3f& r_BN_N,
                               const Eigen::Vector3f& r_SN_N,
                               uint64_t callTime);

   private:
    SunTrackErrorOutput computeSunTrackError(const SunTrackErrorNavAttInputs& nav,
                                             const SunTrackErrorAttRefInputs& ref,
                                             uint64_t callTime) const;

    SunTrackErrorConfig cfg;

    Eigen::Vector3f mnvrAxis_B{Eigen::Vector3f::Zero()};  //!< [-] Eigen axis that we are maneuvering on*/
    float angleStart{};                                   //!< [r] The angle remaining in the attitude maneuver*/
    bool maneuverInitialized{};                           //!< [-] Flag indicating if maneuver has been set*/
    uint64_t mnvrStartTime{};                             //!< [ns] Time at which the maneuver was begun*/
};

#endif
