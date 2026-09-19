#ifndef F32XMERA_MOMENTUM_MANAGEMENT_ALGORITHM_H
#define F32XMERA_MOMENTUM_MANAGEMENT_ALGORITHM_H

#include "momentumManagementTypes.h"
#include "msgPayloadDef/definitions.h"
#include "utilities/fsw/deviceAvailability.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/freestandingIsFinite.hpp"
#include <math.h>
#include <stdint.h>

#include <Eigen/Core>
#include <array>
#include <utility>

/*! @brief Reaction-wheel spin-axis configuration used to compute the net cluster momentum. */
struct MomentumManagementRwArrayConfiguration {
    Eigen::Matrix<float, 3, kMaxNumRw> GsMatrix_B{
        Eigen::Matrix<float, 3, kMaxNumRw>::Zero()};  //!< [-] RW spin axes in body frame, one column per wheel
    Eigen::Vector<float, kMaxNumRw> JsList{Eigen::Vector<float, kMaxNumRw>::Zero()};  //!< [kgm2] RW spin-axis inertias
    std::array<fsw::DeviceAvailability, kMaxNumRw>
        wheelAvailability{};  //!< [-] AVAILABLE / UNAVAILABLE state of each wheel (fixed at reset)
};

/*! @brief Dumping threshold, feedback gains and integration step of the momentum management control law. */
struct MomentumManagementControlParameters {
    float hsMin{};          //!< [Nms] RW cluster momentum below which no dumping is requested
    float K{};              //!< [1/s] proportional gain mapping the stored wheel momentum onto the requested torque
    float Ki{};             //!< [1/s2] integral gain on the accumulated stored momentum (0 disables the integral)
    float integralLimit{};  //!< [Nms2] anti-windup clamp on each component of the momentum integral
    float controlPeriod{};  //!< [s] time between two update() calls, the integration step (only used when Ki > 0)
    Eigen::Matrix3f dumpableProjection_B{
        Eigen::Matrix3f::Identity()};  //!< [-] projector onto the directions the effectors can dump about
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
            FSW_THROW_INVALID_ARGUMENT("momentumManagement: K must be finite and non-negative.");
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
        if (!isValidDumpableProjection(controlParameters.dumpableProjection_B)) {
            FSW_THROW_INVALID_ARGUMENT(
                "momentumManagement: dumpableProjection_B must be a finite, symmetric and idempotent orthogonal "
                "projector that leaves at least one direction dumpable; use the identity when the effectors can "
                "dump about every direction.");
        }
        if (!isValidRwArrayConfiguration(rwArrayConfig)) {
            FSW_THROW_INVALID_ARGUMENT(
                "momentumManagement: the spin axis matrix and spin-axis inertias must be finite, and every "
                "spin axis must be a unit vector.");
        }

        // Normalize the RW spin axes so the momentum sum can rely on exact unit vectors. The inputs are
        // validated (near-)unit, so this only removes rounding.
        MomentumManagementRwArrayConfiguration normalizedRwArrayConfig = rwArrayConfig;
        for (uint32_t i = 0U; i < kMaxNumRw; ++i) {
            normalizedRwArrayConfig.GsMatrix_B.col(i).normalize();
        }

        return {controlParameters, std::move(normalizedRwArrayConfig)};
    }

    static bool isValidHsMin(float hsMin) { return fsw::is_finite(hsMin) && hsMin >= 0.0F; }
    static bool isValidK(float K) { return fsw::is_finite(K) && K >= 0.0F; }
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
    /*! The projector names the directions the effectors can dump about, so it must be a genuine orthogonal
     projector: finite, symmetric and idempotent. The identity says every direction can be dumped, and a plane
     projector I - n*n^T says the single direction n cannot. It must also leave at least one direction
     dumpable: a rank-zero projector is a valid projector but would make the module request nothing, for ever,
     without reporting anything. A zero-filled matrix from a caller that never set this is exactly that. */
    static bool isValidDumpableProjection(const Eigen::Matrix3f& dumpableProjection_B) {
        constexpr float kProjectionTol = 1e-4F;
        constexpr float kMinRank = 0.5F;  // an orthogonal projector's trace is its rank, so this rejects rank 0
        if (!dumpableProjection_B.allFinite()) {
            return false;
        }
        const Eigen::Matrix3f asymmetry = dumpableProjection_B - dumpableProjection_B.transpose();
        if (asymmetry.reshaped().stableNorm() > kProjectionTol) {
            return false;
        }
        const Eigen::Matrix3f idempotencyError = (dumpableProjection_B * dumpableProjection_B) - dumpableProjection_B;
        if (idempotencyError.reshaped().stableNorm() > kProjectionTol) {
            return false;
        }
        return dumpableProjection_B.trace() >= kMinRank;
    }

    static bool isValidRwArrayConfiguration(const MomentumManagementRwArrayConfiguration& rwArrayConfig) {
        if (!rwArrayConfig.GsMatrix_B.allFinite() || !rwArrayConfig.JsList.allFinite()) {
            return false;
        }
        // Every wheel slot describes a wheel, so every spin axis must be (close to) a unit vector; they are
        // normalized exactly on construction.
        constexpr float kUnitNormTol = 1e-3F;
        for (uint32_t i = 0U; i < kMaxNumRw; ++i) {
            if (fabsf(rwArrayConfig.GsMatrix_B.col(i).stableNorm() - 1.0F) > kUnitNormTol) {
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
 * @brief Assesses the net reaction wheel momentum and computes the torque needed to dump it.
 *
 * The control law is proportional-integral on the stored momentum, gated by the dumping threshold, so the
 * algorithm carries the integrator state between updates. A momentum below the threshold ends the dump and
 * clears that state. Call reInitialize() to re-seed it directly. Only the momentum the effectors can dump
 * reaches the law; dumpableProjection_B says which directions those are.
 */
class MomentumManagementAlgorithm final {
   public:
    explicit MomentumManagementAlgorithm(const MomentumManagementConfig& config);

    //! Install the validated configuration; does not touch runtime state.
    void setConfig(const MomentumManagementConfig& config);

    //! Re-seed the runtime integrator state to its initial values.
    void reInitialize();

    //! [Nm] Requested body-frame torque that dumps the stored wheel momentum for the supplied wheel speeds.
    Eigen::Vector3f update(const Eigen::Vector<float, kMaxNumRw>& wheelSpeeds);

   private:
    MomentumManagementConfig cfg;  //!< [-] validated configuration (control parameters, RW array config)
    Eigen::Vector3f hsInt_B{Eigen::Vector3f::Zero()};  //!< [Nms2] integral of the dumpable RW momentum, B frame
    Eigen::Vector3f priorHsDumpable_B{
        Eigen::Vector3f::Zero()};  //!< [Nms] dumpable RW cluster momentum from the previous update
};

#endif
