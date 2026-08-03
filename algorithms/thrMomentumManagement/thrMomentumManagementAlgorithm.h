#ifndef F32XMERA_THR_MOMENTUM_MANAGEMENT_ALGORITHM_H
#define F32XMERA_THR_MOMENTUM_MANAGEMENT_ALGORITHM_H

#include "thrMomentumManagementTypes.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/freestandingIsFinite.hpp"
#include <math.h>
#include <stdint.h>

#include <Eigen/Core>
#include <optional>

inline constexpr uint32_t kMaxNumRw = THR_MOMENTUM_MANAGEMENT_MAX_NUM_RW;  //!< [-] maximum number of reaction wheels

/*! @brief Reaction-wheel spin-axis configuration used to compute the net cluster momentum. */
struct ThrMomentumManagementRwArrayConfiguration {
    uint32_t numRW{};  //!< [-] number of reaction wheels on the vehicle
    Eigen::Matrix<float, 3, kMaxNumRw> GsMatrix_B{
        Eigen::Matrix<float, 3, kMaxNumRw>::Zero()};  //!< [-] RW spin axes in body frame, one column per wheel
    Eigen::Vector<float, kMaxNumRw> JsList{Eigen::Vector<float, kMaxNumRw>::Zero()};  //!< [kgm2] RW spin-axis inertias
};

/*!
 * @brief Validated configuration for the RW momentum management algorithm.
 *
 * Bundles the momentum dumping threshold and the reaction-wheel spin-axis configuration. An instance can only
 * exist if the threshold is finite and non-negative, and if the reaction-wheel count does not exceed the
 * compile-time maximum with finite spin axes and inertias and each spin axis a unit vector. Construct via
 * ThrMomentumManagementConfig::create(...).
 */
class ThrMomentumManagementConfig final {
   public:
    static ThrMomentumManagementConfig create(float hsMin,
                                              const ThrMomentumManagementRwArrayConfiguration& rwArrayConfig) {
        if (!isValidHsMin(hsMin)) {
            FSW_THROW_INVALID_ARGUMENT(
                "thrMomentumManagement: hsMin (minimum RW cluster momentum for dumping) must be finite and "
                "non-negative.");
        }
        if (!isValidRwArrayConfiguration(rwArrayConfig)) {
            FSW_THROW_INVALID_ARGUMENT(
                "thrMomentumManagement: rwArrayConfig.numRW must not exceed the compile-time maximum, the spin "
                "axis matrix and spin-axis inertias must be finite, and each spin axis must be a unit vector.");
        }

        // Normalize the RW spin axes so the momentum sum can rely on exact unit vectors. The inputs are
        // validated (near-)unit, so this only removes rounding.
        ThrMomentumManagementRwArrayConfiguration normalizedRwArrayConfig = rwArrayConfig;
        for (uint32_t i = 0U; i < normalizedRwArrayConfig.numRW; ++i) {
            normalizedRwArrayConfig.GsMatrix_B.col(i).normalize();
        }

        return {hsMin, normalizedRwArrayConfig};
    }

    static bool isValidHsMin(float hsMin) { return fsw::is_finite(hsMin) && hsMin >= 0.0F; }

    static bool isValidRwArrayConfiguration(const ThrMomentumManagementRwArrayConfiguration& rwArrayConfig) {
        if (rwArrayConfig.numRW > kMaxNumRw || !rwArrayConfig.GsMatrix_B.allFinite() ||
            !rwArrayConfig.JsList.allFinite()) {
            return false;
        }
        // Each spin axis must be (close to) a unit vector; they are normalized exactly on construction.
        constexpr float kUnitNormTol = 1e-3F;
        for (uint32_t i = 0U; i < rwArrayConfig.numRW; ++i) {
            if (fabsf(rwArrayConfig.GsMatrix_B.col(i).norm() - 1.0F) > kUnitNormTol) {
                return false;
            }
        }
        return true;
    }

    float getHsMin() const { return this->hsMin; }
    const ThrMomentumManagementRwArrayConfiguration& getRwArrayConfiguration() const { return this->rwArrayConfig; }

   private:
    ThrMomentumManagementConfig(float hsMin, const ThrMomentumManagementRwArrayConfiguration& rwArrayConfig)
        : hsMin(hsMin), rwArrayConfig(rwArrayConfig) {}

    float hsMin;                                              //!< [Nms] minimum RW cluster momentum for dumping
    ThrMomentumManagementRwArrayConfiguration rwArrayConfig;  //!< [-] RW spin axes and spin-axis inertias
};

/*!
 * @brief Assesses the net reaction wheel momentum and computes the angular momentum change needed to dump it.
 *
 * The momentum check is a one-shot: it runs on the first update() after a re-initialization and is then
 * disarmed, so a dump is requested at most once per re-initialization. Call reInitialize() to re-arm it.
 */
class ThrMomentumManagementAlgorithm final {
   public:
    explicit ThrMomentumManagementAlgorithm(const ThrMomentumManagementConfig& config);

    //! Install the validated configuration; does not touch runtime state.
    void setConfig(const ThrMomentumManagementConfig& config);

    //! Re-arm the one-shot momentum dumping request.
    void reInitialize();

    //! [Nms] Requested body-frame angular momentum change, or nullopt once the one-shot check has been spent.
    std::optional<Eigen::Vector3f> update(const Eigen::Vector<float, kMaxNumRw>& wheelSpeeds);

   private:
    ThrMomentumManagementConfig cfg;  //!< [-] validated configuration (dumping threshold, RW array config)
    bool dumpRequested{};             //!< [-] true while the one-shot momentum check is still pending
};

#endif
