#ifndef F32XMERA_MRP_FEEDBACK_ALGORITHM_H
#define F32XMERA_MRP_FEEDBACK_ALGORITHM_H

#include "../msgPayloadDef/definitions.h"
#include "msgPayloadDef/RWAvailabilityMsgPayload.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/freestandingIsFinite.hpp"
#include "utilities/fsw/validInertiaCheck.h"

#include <math.h>
#include <stdint.h>
#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <optional>

enum class ControlLawType { NORMAL = 0, SIMPLE_INTEGRAL = 1 };

/*! Commanded control torque and its integral-feedback component. */
struct MrpFeedbackOutput {
    Eigen::Vector3f controlTorque = Eigen::Vector3f::Zero();           //!< [N*m] commanded control torque Lr
    Eigen::Vector3f integralFeedbackTorque = Eigen::Vector3f::Zero();  //!< [N*m] integral feedback torque Li
};

/*! Struct containing the guidance inputs needed by the algorithm. */
struct MrpFeedbackInputGuidance {
    Eigen::Vector3f sigma_BR = Eigen::Vector3f::Zero();
    Eigen::Vector3f omega_BR_B = Eigen::Vector3f::Zero();
    Eigen::Vector3f omega_RN_B = Eigen::Vector3f::Zero();
    Eigen::Vector3f domega_RN_B = Eigen::Vector3f::Zero();
};

/*! Struct containing the reaction wheel inputs needed by the algorithm. */
struct MrpFeedbackInputRwData {
    Eigen::Matrix<float, 3, RW_EFF_CNT> GsMatrix_B = Eigen::Matrix<float, 3, RW_EFF_CNT>::Zero();
    std::array<float, RW_EFF_CNT> JsList{};
    std::array<int32_t, RW_EFF_CNT> wheelAvailability{};  //!< per-wheel AVAILABLE/UNAVAILABLE (fixed at reset)
    uint32_t numRW{};
};

/*! Feedback control gains, control-law variant, and update period. */
struct MrpFeedbackControlParameters {
    float K{};                                              //!< [N*m] proportional gain on the MRP error
    float P{};                                              //!< [N*m*s] rate-error feedback gain
    float Ki{};                                             //!< [N*m] integral feedback gain (0 disables the integral)
    float integralLimit{};                                  //!< [N*m*s] anti-windup clamp on the integrated MRP error
    ControlLawType controlLawType{ControlLawType::NORMAL};  //!< control-law variant
    float controlPeriod{};                                  //!< [s] time between two algorithm update calls
};

class MrpFeedbackConfig final {
   public:
    static constexpr uint32_t kMaxNumRw = RW_EFF_CNT;  //!< [-] compile-time maximum number of reaction wheels

    static MrpFeedbackConfig create(const MrpFeedbackControlParameters& controlParameters,
                                    const Eigen::Vector3f& knownTorquePntB_B,
                                    const Eigen::Matrix3f& ISCPntB_B,
                                    const std::optional<MrpFeedbackInputRwData>& rwConfiguration = std::nullopt) {
        if (!isValidK(controlParameters.K)) {
            FSW_THROW_INVALID_ARGUMENT("mrpFeedback: K must be finite and >= 0");
        }
        if (!isValidP(controlParameters.P)) {
            FSW_THROW_INVALID_ARGUMENT("mrpFeedback: P must be finite and >= 0");
        }
        if (!isValidKi(controlParameters.Ki)) {
            FSW_THROW_INVALID_ARGUMENT("mrpFeedback: Ki must be finite and >= 0");
        }
        if (!isValidIntegralLimit(controlParameters.integralLimit)) {
            FSW_THROW_INVALID_ARGUMENT("mrpFeedback: integralLimit must be finite and >= 0");
        }
        if (!isValidControlLawType(controlParameters.controlLawType)) {
            FSW_THROW_INVALID_ARGUMENT("mrpFeedback: controlLawType must be NORMAL or SIMPLE_INTEGRAL");
        }
        if (!isValidControlPeriod(controlParameters.controlPeriod)) {
            FSW_THROW_INVALID_ARGUMENT("mrpFeedback: controlPeriod must be finite and > 0");
        }
        if (!isValidKnownTorque(knownTorquePntB_B)) {
            FSW_THROW_INVALID_ARGUMENT("mrpFeedback: knownTorquePntB_B must be finite");
        }
        if (!isValidInertia(ISCPntB_B)) {
            FSW_THROW_INVALID_ARGUMENT("mrpFeedback: ISCPntB_B must be a valid inertia matrix");
        }
        if (rwConfiguration.has_value() && !isValidRwConfiguration(*rwConfiguration)) {
            FSW_THROW_INVALID_ARGUMENT(
                "mrpFeedback: rwConfiguration.numRW must not exceed the compile-time maximum, the spin-axis matrix "
                "and wheel inertias must be finite, and each active spin axis must be a unit vector");
        }

        // Normalize the validated (near-)unit spin axes so downstream code can rely on exact unit vectors; this
        // only removes rounding. Inactive columns (index >= numRW) are left untouched.
        std::optional<MrpFeedbackInputRwData> normalizedRwConfiguration = rwConfiguration;
        if (normalizedRwConfiguration.has_value()) {
            for (uint32_t i = 0U; i < normalizedRwConfiguration->numRW; ++i) {
                normalizedRwConfiguration->GsMatrix_B.col(static_cast<int>(i)).normalize();
            }
        }
        return {controlParameters, knownTorquePntB_B, ISCPntB_B, normalizedRwConfiguration};
    }

    static bool isValidK(float K) { return fsw::is_finite(K) && K >= 0.0F; }
    static bool isValidP(float P) { return fsw::is_finite(P) && P >= 0.0F; }
    static bool isValidKi(float Ki) { return fsw::is_finite(Ki) && Ki >= 0.0F; }
    static bool isValidIntegralLimit(float integralLimit) {
        return fsw::is_finite(integralLimit) && integralLimit >= 0.0F;
    }
    static bool isValidControlPeriod(float controlPeriod) {
        return fsw::is_finite(controlPeriod) && controlPeriod > 0.0F;
    }
    static bool isValidControlLawType(ControlLawType controlLawType) {
        return controlLawType == ControlLawType::NORMAL || controlLawType == ControlLawType::SIMPLE_INTEGRAL;
    }
    static bool isValidKnownTorque(const Eigen::Vector3f& knownTorquePntB_B) { return knownTorquePntB_B.allFinite(); }
    static bool isValidInertia(const Eigen::Matrix3f& ISCPntB_B) { return inertiaIsValid(ISCPntB_B); }
    static bool isValidRwConfiguration(const MrpFeedbackInputRwData& rwConfiguration) {
        if (rwConfiguration.numRW > kMaxNumRw || !rwConfiguration.GsMatrix_B.allFinite()) {
            return false;
        }
        if (!std::ranges::all_of(rwConfiguration.JsList, [](float Js) { return fsw::is_finite(Js); })) {
            return false;
        }
        // Each active spin axis must be (close to) a unit vector.
        constexpr float kUnitNormTol = 1e-3F;
        for (uint32_t i = 0U; i < rwConfiguration.numRW; ++i) {
            if (fabsf(rwConfiguration.GsMatrix_B.col(static_cast<int>(i)).stableNorm() - 1.0F) > kUnitNormTol) {
                return false;
            }
        }
        return true;
    }

    const MrpFeedbackControlParameters& getControlParameters() const { return this->controlParameters; }
    const Eigen::Vector3f& getKnownTorquePntB_B() const { return this->knownTorquePntB_B; }
    const Eigen::Matrix3f& getSpacecraftInertia() const { return this->ISCPntB_B; }
    const std::optional<MrpFeedbackInputRwData>& getRwConfiguration() const { return this->rwConfiguration; }

   private:
    MrpFeedbackConfig(const MrpFeedbackControlParameters& controlParameters,
                      const Eigen::Vector3f& knownTorquePntB_B,
                      const Eigen::Matrix3f& ISCPntB_B,
                      const std::optional<MrpFeedbackInputRwData>& rwConfiguration)
        : controlParameters(controlParameters),
          knownTorquePntB_B(knownTorquePntB_B),
          ISCPntB_B(ISCPntB_B),
          rwConfiguration(rwConfiguration) {}

    MrpFeedbackControlParameters controlParameters;
    Eigen::Vector3f knownTorquePntB_B;
    Eigen::Matrix3f ISCPntB_B;
    std::optional<MrpFeedbackInputRwData> rwConfiguration;
};

/*! @brief Data configuration structure for the MRP feedback attitude control routine. */
class MrpFeedbackAlgorithm final {
   public:
    explicit MrpFeedbackAlgorithm(const MrpFeedbackConfig& config);

    void setConfig(const MrpFeedbackConfig& config);

    //! Reset the integrating runtime state (zero the integral of the MRP tracking error).
    void reInitialize();

    MrpFeedbackOutput update(const MrpFeedbackInputGuidance& attGuidInput,
                             const std::array<float, RW_EFF_CNT>& wheelSpeeds);

   private:
    MrpFeedbackConfig cfg;
    Eigen::Vector3f int_sigma{};  //!< [s] integral of the MPR attitude error
};

#endif
