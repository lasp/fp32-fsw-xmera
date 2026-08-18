#ifndef F32XMERA_THR_MOMENTUM_MANAGEMENT_ALGORITHM_H
#define F32XMERA_THR_MOMENTUM_MANAGEMENT_ALGORITHM_H

#include "thrMomentumManagementTypes.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/freestandingIsFinite.hpp"
#include <math.h>
#include <stdint.h>

#include <Eigen/Core>
#include <utility>

inline constexpr uint32_t kMaxNumRw = THR_MOMENTUM_MANAGEMENT_MAX_NUM_RW;  //!< [-] maximum number of reaction wheels

/*! @brief Reaction-wheel spin-axis configuration used to compute the net cluster momentum. */
struct ThrMomentumManagementRwArrayConfiguration {
    uint32_t numRW{};  //!< [-] number of reaction wheels on the vehicle
    Eigen::Matrix<float, 3, kMaxNumRw> GsMatrix_B{
        Eigen::Matrix<float, 3, kMaxNumRw>::Zero()};  //!< [-] RW spin axes in body frame, one column per wheel
    Eigen::Vector<float, kMaxNumRw> JsList{Eigen::Vector<float, kMaxNumRw>::Zero()};  //!< [kgm2] RW spin-axis inertias
};

/*! @brief Dumping threshold and feedback gain of the momentum management control law. */
struct ThrMomentumManagementControlParameters {
    float hsMin{};  //!< [Nms] RW cluster momentum below which no dumping is requested
    float K{};      //!< [1/s] proportional gain mapping the excess wheel momentum onto the requested torque
};

/*! @brief Validated configuration for the RW momentum management algorithm. */
class ThrMomentumManagementConfig final {
   public:
    static ThrMomentumManagementConfig create(const ThrMomentumManagementControlParameters& controlParameters,
                                              const ThrMomentumManagementRwArrayConfiguration& rwArrayConfig) {
        if (!isValidHsMin(controlParameters.hsMin)) {
            FSW_THROW_INVALID_ARGUMENT(
                "thrMomentumManagement: hsMin (minimum RW cluster momentum for dumping) must be finite and "
                "non-negative.");
        }
        if (!isValidK(controlParameters.K)) {
            FSW_THROW_INVALID_ARGUMENT("thrMomentumManagement: K must be finite and positive.");
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

        return {controlParameters, std::move(normalizedRwArrayConfig)};
    }

    static bool isValidHsMin(float hsMin) { return fsw::is_finite(hsMin) && hsMin >= 0.0F; }
    static bool isValidK(float K) { return fsw::is_finite(K) && K > 0.0F; }

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

    const ThrMomentumManagementControlParameters& getControlParameters() const { return this->controlParameters; }
    const ThrMomentumManagementRwArrayConfiguration& getRwArrayConfiguration() const { return this->rwArrayConfig; }

   private:
    ThrMomentumManagementConfig(const ThrMomentumManagementControlParameters& controlParameters,
                                ThrMomentumManagementRwArrayConfiguration rwArrayConfig)
        : controlParameters(controlParameters), rwArrayConfig(std::move(rwArrayConfig)) {}

    ThrMomentumManagementControlParameters controlParameters;  //!< [-] dumping threshold and feedback gain
    ThrMomentumManagementRwArrayConfiguration rwArrayConfig;   //!< [-] RW spin axes and spin-axis inertias
};

/*! @brief Assesses the net reaction wheel momentum and computes the torque needed to dump its excess. */
class ThrMomentumManagementAlgorithm final {
   public:
    explicit ThrMomentumManagementAlgorithm(const ThrMomentumManagementConfig& config);

    //! Install the validated configuration.
    void setConfig(const ThrMomentumManagementConfig& config);

    //! [Nm] Requested body-frame torque that dumps the excess wheel momentum for the supplied wheel speeds.
    Eigen::Vector3f update(const Eigen::Vector<float, kMaxNumRw>& wheelSpeeds) const;

   private:
    ThrMomentumManagementConfig cfg;  //!< [-] validated configuration (control parameters, RW array config)
};

#endif
