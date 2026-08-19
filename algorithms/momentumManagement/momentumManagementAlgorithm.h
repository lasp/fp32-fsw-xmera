#ifndef F32XMERA_MOMENTUM_MANAGEMENT_ALGORITHM_H
#define F32XMERA_MOMENTUM_MANAGEMENT_ALGORITHM_H

#include "momentumManagementTypes.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/freestandingIsFinite.hpp"
#include <math.h>
#include <stdint.h>

#include <Eigen/Core>
#include <utility>

inline constexpr uint32_t kMaxNumRw = MOMENTUM_MANAGEMENT_MAX_NUM_RW;  //!< [-] maximum number of reaction wheels

/*! @brief Reaction-wheel spin-axis configuration used to compute the net cluster momentum. */
struct MomentumManagementRwArrayConfiguration {
    uint32_t numRW{};  //!< [-] number of reaction wheels on the vehicle
    Eigen::Matrix<float, 3, kMaxNumRw> GsMatrix_B{
        Eigen::Matrix<float, 3, kMaxNumRw>::Zero()};  //!< [-] RW spin axes in body frame, one column per wheel
    Eigen::Vector<float, kMaxNumRw> JsList{Eigen::Vector<float, kMaxNumRw>::Zero()};  //!< [kgm2] RW spin-axis inertias
};

/*! @brief Dumping threshold, feedback gains and integration step of the momentum management control law. */
struct MomentumManagementControlParameters {
    float hsMin{};          //!< [Nms] RW cluster momentum below which no dumping is requested
    float K{};              //!< [1/s] proportional gain mapping the excess wheel momentum onto the requested torque
    float Ki{};             //!< [1/s2] integral gain on the accumulated excess momentum (0 disables the integral)
    float integralLimit{};  //!< [Nms2] anti-windup clamp on each component of the excess-momentum integral
    float controlPeriod{};  //!< [s] time between two update() calls, the integration step (only used when Ki > 0)
};

/*! @brief Validated configuration for the RW momentum management algorithm. */
class MomentumManagementConfig final {
   public:
    static MomentumManagementConfig create(const MomentumManagementControlParameters& controlParameters,
                                           const MomentumManagementRwArrayConfiguration& rwArrayConfig) {
        if (!isValidHsMin(controlParameters.hsMin)) {
            FSW_THROW_INVALID_ARGUMENT(
                "momentumManagement: hsMin (minimum RW cluster momentum for dumping) must be finite and "
                "non-negative.");
        }
        if (!isValidK(controlParameters.K)) {
            FSW_THROW_INVALID_ARGUMENT("momentumManagement: K must be finite and positive.");
        }
        if (!isValidKi(controlParameters.Ki)) {
            FSW_THROW_INVALID_ARGUMENT("momentumManagement: Ki must be finite and non-negative.");
        }
        if (!isValidIntegralLimit(controlParameters.integralLimit, controlParameters.Ki)) {
            FSW_THROW_INVALID_ARGUMENT(
                "momentumManagement: integralLimit must be finite and non-negative, and positive when Ki > 0.");
        }
        if (!isValidControlPeriod(controlParameters.controlPeriod, controlParameters.Ki)) {
            FSW_THROW_INVALID_ARGUMENT(
                "momentumManagement: controlPeriod must be finite and non-negative, and positive when Ki > 0.");
        }
        if (!isValidRwArrayConfiguration(rwArrayConfig)) {
            FSW_THROW_INVALID_ARGUMENT(
                "momentumManagement: rwArrayConfig.numRW must not exceed the compile-time maximum, the spin "
                "axis matrix and spin-axis inertias must be finite, and each spin axis must be a unit vector.");
        }

        // Normalize the RW spin axes so the momentum sum can rely on exact unit vectors. The inputs are
        // validated (near-)unit, so this only removes rounding.
        MomentumManagementRwArrayConfiguration normalizedRwArrayConfig = rwArrayConfig;
        for (uint32_t i = 0U; i < normalizedRwArrayConfig.numRW; ++i) {
            normalizedRwArrayConfig.GsMatrix_B.col(i).normalize();
        }

        return {controlParameters, std::move(normalizedRwArrayConfig)};
    }

    static bool isValidHsMin(float hsMin) { return fsw::is_finite(hsMin) && hsMin >= 0.0F; }
    static bool isValidK(float K) { return fsw::is_finite(K) && K > 0.0F; }
    static bool isValidKi(float Ki) { return fsw::is_finite(Ki) && Ki >= 0.0F; }
    /*! A zero limit is only allowed when the integral term is switched off (Ki == 0). */
    static bool isValidIntegralLimit(float integralLimit, float Ki) {
        return fsw::is_finite(integralLimit) && integralLimit >= 0.0F && (Ki == 0.0F || integralLimit > 0.0F);
    }
    /*! Only the integral term consumes the control period, so it may be left at zero when Ki == 0. It must stay
     finite regardless: a non-finite step would poison the integral, and Ki * NaN is NaN even for Ki == 0. */
    static bool isValidControlPeriod(float controlPeriod, float Ki) {
        return fsw::is_finite(controlPeriod) && controlPeriod >= 0.0F && (Ki == 0.0F || controlPeriod > 0.0F);
    }

    static bool isValidRwArrayConfiguration(const MomentumManagementRwArrayConfiguration& rwArrayConfig) {
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

    const MomentumManagementControlParameters& getControlParameters() const { return this->controlParameters; }
    const MomentumManagementRwArrayConfiguration& getRwArrayConfiguration() const { return this->rwArrayConfig; }

   private:
    MomentumManagementConfig(const MomentumManagementControlParameters& controlParameters,
                             MomentumManagementRwArrayConfiguration rwArrayConfig)
        : controlParameters(controlParameters), rwArrayConfig(std::move(rwArrayConfig)) {}

    MomentumManagementControlParameters controlParameters;  //!< [-] dumping threshold and feedback gain
    MomentumManagementRwArrayConfiguration rwArrayConfig;   //!< [-] RW spin axes and spin-axis inertias
};

/*!
 * @brief Assesses the net reaction wheel momentum and computes the torque needed to dump its excess.
 *
 * The control law is proportional-integral on the momentum held above the dumping threshold, so the algorithm
 * carries the integrator state between updates. Call reInitialize() to re-seed it.
 */
class MomentumManagementAlgorithm final {
   public:
    explicit MomentumManagementAlgorithm(const MomentumManagementConfig& config);

    //! Install the validated configuration; does not touch runtime state.
    void setConfig(const MomentumManagementConfig& config);

    //! Re-seed the runtime integrator state to its initial values.
    void reInitialize();

    //! [Nm] Requested body-frame torque that dumps the excess wheel momentum for the supplied wheel speeds.
    Eigen::Vector3f update(const Eigen::Vector<float, kMaxNumRw>& wheelSpeeds);

   private:
    MomentumManagementConfig cfg;  //!< [-] validated configuration (control parameters, RW array config)
    Eigen::Vector3f hsInt_B{Eigen::Vector3f::Zero()};  //!< [Nms2] integral of the excess RW momentum, B frame
    Eigen::Vector3f priorHsExcess_B{
        Eigen::Vector3f::Zero()};  //!< [Nms] excess RW momentum from the previous update, B frame
};

#endif
