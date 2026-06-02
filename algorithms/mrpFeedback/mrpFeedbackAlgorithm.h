#ifndef F32XMERA_MRP_FEEDBACK_ALGORITHM_H
#define F32XMERA_MRP_FEEDBACK_ALGORITHM_H

#include <cstdint>

#include "mrpFeedbackTypes.h"
#include "msgPayloadDef/AttGuidMsgF32Payload.h"
#include "msgPayloadDef/CmdTorqueBodyMsgF32Payload.h"
#include "msgPayloadDef/RWArrayConfigMsgF32Payload.h"
#include "msgPayloadDef/RWAvailabilityMsgPayload.h"
#include "msgPayloadDef/RWSpeedMsgF32Payload.h"
#include "msgPayloadDef/VehicleConfigMsgF32Payload.h"
#include "utilities/freestandingInvalidArgument.h"
#include "utilities/validInertiaCheck.h"

#include <Eigen/Core>

enum class ControlLawType { NORMAL = 0, SIMPLE_INTEGRAL = 1 };

struct MrpFeedbackOutput {
    CmdTorqueBodyMsgF32Payload controlOut{};      //!< control torque output
    CmdTorqueBodyMsgF32Payload intFeedbackOut{};  //!< integral feedback torque output
};

class MrpFeedbackConfig final {
   public:
    static MrpFeedbackConfig create(float K,
                                    float P,
                                    float Ki,
                                    float integralLimit,
                                    ControlLawType controlLawType,
                                    const Eigen::Vector3f& knownTorquePntB_B,
                                    const Eigen::Matrix3f& ISCPntB_B) {
        if (!isValidK(K)) {
            FSW_THROW_INVALID_ARGUMENT("mrpFeedback: K must be >= 0");
        }
        if (!isValidP(P)) {
            FSW_THROW_INVALID_ARGUMENT("mrpFeedback: P must be >= 0");
        }
        if (!isValidKi(Ki)) {
            FSW_THROW_INVALID_ARGUMENT("mrpFeedback: Ki must be >= 0");
        }
        if (!isValidIntegralLimit(integralLimit)) {
            FSW_THROW_INVALID_ARGUMENT("mrpFeedback: integralLimit must be >= 0");
        }
        if (!isValidControlLawType(controlLawType)) {
            FSW_THROW_INVALID_ARGUMENT("mrpFeedback: controlLawType must be NORMAL or SIMPLE_INTEGRAL");
        }
        if (!isValidKnownTorquePntB_B(knownTorquePntB_B)) {
            FSW_THROW_INVALID_ARGUMENT("mrpFeedback: knownTorquePntB_B must be finite");
        }
        if (!isValidISCPntB_B(ISCPntB_B)) {
            FSW_THROW_INVALID_ARGUMENT("mrpFeedback: ISCPntB_B must be a valid inertia tensor");
        }
        return {K, P, Ki, integralLimit, controlLawType, knownTorquePntB_B, ISCPntB_B};
    }

    static bool isValidK(float K) { return K >= 0.0F; }
    static bool isValidP(float P) { return P >= 0.0F; }
    static bool isValidKi(float Ki) { return Ki >= 0.0F; }
    static bool isValidIntegralLimit(float limit) { return limit >= 0.0F; }
    static bool isValidControlLawType(ControlLawType t) {
        return t == ControlLawType::NORMAL || t == ControlLawType::SIMPLE_INTEGRAL;
    }
    static bool isValidKnownTorquePntB_B(const Eigen::Vector3f& torque) { return torque.allFinite(); }
    static bool isValidISCPntB_B(const Eigen::Matrix3f& inertia) { return inertiaIsValid(inertia); }

    float getK() const { return K; }
    float getP() const { return P; }
    float getKi() const { return Ki; }
    float getIntegralLimit() const { return integralLimit; }
    ControlLawType getControlLawType() const { return controlLawType; }
    Eigen::Vector3f getKnownTorquePntB_B() const { return knownTorquePntB_B; }
    Eigen::Matrix3f getISCPntB_B() const { return ISCPntB_B; }

   private:
    MrpFeedbackConfig(float K,
                      float P,
                      float Ki,
                      float integralLimit,
                      ControlLawType controlLawType,
                      const Eigen::Vector3f& knownTorquePntB_B,
                      const Eigen::Matrix3f& ISCPntB_B)
        : K(K),
          P(P),
          Ki(Ki),
          integralLimit(integralLimit),
          controlLawType(controlLawType),
          knownTorquePntB_B(knownTorquePntB_B),
          ISCPntB_B(ISCPntB_B) {}

    float K;
    float P;
    float Ki;
    float integralLimit;
    ControlLawType controlLawType;
    Eigen::Vector3f knownTorquePntB_B;
    Eigen::Matrix3f ISCPntB_B;
};

/*! @brief Data configuration structure for the MRP feedback attitude control routine. */
class MrpFeedbackAlgorithm final {
   public:
    explicit MrpFeedbackAlgorithm(MrpFeedbackConfig config);

    void setConfig(const MrpFeedbackConfig& config);

    void reset(const RWArrayConfigMsgF32Payload& rwConfigMsg, bool rwIsLinked);
    MrpFeedbackOutput update(uint64_t callTime,
                             const AttGuidMsgF32Payload& guidCmd,
                             const RWSpeedMsgF32Payload& wheelSpeeds,
                             const RWAvailabilityMsgPayload& wheelsAvailability);

   private:
    MrpFeedbackConfig cfg;
    uint64_t priorTime{};                         //!< [ns]      Last time the attitude control is called
    Eigen::Vector3f int_sigma{};                  //!< [s] integral of the MPR attitude error
    RWArrayConfigMsgF32Payload rwConfigParams{};  //!< RW config snapshot taken at reset() time
};

#endif
