#ifndef F32XMERA_MRP_FEEDBACK_ALGORITHM_H
#define F32XMERA_MRP_FEEDBACK_ALGORITHM_H

#include <algorithm>
#include <array>
#include <cstdint>

#include "mrpFeedbackTypes.h"
#include "msgPayloadDef/definitions.h"  // RW_EFF_CNT
#include "utilities/freestandingInvalidArgument.h"
#include "utilities/freestandingIsFinite.hpp"
#include "utilities/validInertiaCheck.h"

#include <Eigen/Core>

enum class ControlLawType { NORMAL = 0, SIMPLE_INTEGRAL = 1 };

/// Per-cycle attitude/rate tracking error (was AttGuidMsgF32Payload at the interface).
struct MrpFeedbackGuidInput {
    Eigen::Vector3f sigma_BR = Eigen::Vector3f::Zero();    //!< attitude error (MRP) of B relative to R
    Eigen::Vector3f omega_BR_B = Eigen::Vector3f::Zero();  //!< [r/s] body rate error of B relative to R, B frame
    Eigen::Vector3f omega_RN_B = Eigen::Vector3f::Zero();  //!< [r/s] reference rate of R relative to N, B frame
    Eigen::Vector3f domega_RN_B =
        Eigen::Vector3f::Zero();  //!< [r/s^2] reference acceleration of R relative to N, B frame
};

struct MrpFeedbackOutput {
    Eigen::Vector3f controlTorque = Eigen::Vector3f::Zero();      //!< [N*m] required control torque Lr
    Eigen::Vector3f intFeedbackTorque = Eigen::Vector3f::Zero();  //!< [N*m] integral feedback torque Li
};

class MrpFeedbackConfig final {
   public:
    static MrpFeedbackConfig create(float K,
                                    float P,
                                    float Ki,
                                    float integralLimit,
                                    ControlLawType controlLawType,
                                    const Eigen::Vector3f& knownTorquePntB_B,
                                    const Eigen::Matrix3f& ISCPntB_B,
                                    int32_t numRW,
                                    const Eigen::Matrix<float, 3, RW_EFF_CNT>& Gs_B,
                                    const std::array<float, RW_EFF_CNT>& JsList) {
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
        if (!isValidNumRW(numRW)) {
            FSW_THROW_INVALID_ARGUMENT("mrpFeedback: numRW must be in [0, RW_EFF_CNT]");
        }
        if (!isValidGs_B(Gs_B)) {
            FSW_THROW_INVALID_ARGUMENT("mrpFeedback: Gs_B must be finite");
        }
        if (!isValidJsList(JsList)) {
            FSW_THROW_INVALID_ARGUMENT("mrpFeedback: JsList must be finite");
        }
        return {K, P, Ki, integralLimit, controlLawType, knownTorquePntB_B, ISCPntB_B, numRW, Gs_B, JsList};
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
    static bool isValidNumRW(int32_t numRW) { return numRW >= 0 && numRW <= RW_EFF_CNT; }
    static bool isValidGs_B(const Eigen::Matrix<float, 3, RW_EFF_CNT>& Gs_B) { return Gs_B.allFinite(); }
    static bool isValidJsList(const std::array<float, RW_EFF_CNT>& JsList) {
        return std::all_of(JsList.begin(), JsList.end(), [](float j) { return fsw::is_finite(j); });
    }

    float getK() const { return K; }
    float getP() const { return P; }
    float getKi() const { return Ki; }
    float getIntegralLimit() const { return integralLimit; }
    ControlLawType getControlLawType() const { return controlLawType; }
    Eigen::Vector3f getKnownTorquePntB_B() const { return knownTorquePntB_B; }
    Eigen::Matrix3f getISCPntB_B() const { return ISCPntB_B; }
    int32_t getNumRW() const { return numRW; }
    const Eigen::Matrix<float, 3, RW_EFF_CNT>& getGs_B() const { return Gs_B; }
    const std::array<float, RW_EFF_CNT>& getJsList() const { return JsList; }

   private:
    MrpFeedbackConfig(float K,
                      float P,
                      float Ki,
                      float integralLimit,
                      ControlLawType controlLawType,
                      const Eigen::Vector3f& knownTorquePntB_B,
                      const Eigen::Matrix3f& ISCPntB_B,
                      int32_t numRW,
                      const Eigen::Matrix<float, 3, RW_EFF_CNT>& Gs_B,
                      const std::array<float, RW_EFF_CNT>& JsList)
        : K(K),
          P(P),
          Ki(Ki),
          integralLimit(integralLimit),
          controlLawType(controlLawType),
          knownTorquePntB_B(knownTorquePntB_B),
          ISCPntB_B(ISCPntB_B),
          numRW(numRW),
          Gs_B(Gs_B),
          JsList(JsList) {}

    float K;
    float P;
    float Ki;
    float integralLimit;
    ControlLawType controlLawType;
    Eigen::Vector3f knownTorquePntB_B;
    Eigen::Matrix3f ISCPntB_B;
    int32_t numRW;
    Eigen::Matrix<float, 3, RW_EFF_CNT> Gs_B;
    std::array<float, RW_EFF_CNT> JsList;
};

/*! @brief Data configuration structure for the MRP feedback attitude control routine. */
class MrpFeedbackAlgorithm final {
   public:
    explicit MrpFeedbackAlgorithm(MrpFeedbackConfig config);

    void setConfig(const MrpFeedbackConfig& config);

    void reset();
    MrpFeedbackOutput update(uint64_t callTime,
                             const MrpFeedbackGuidInput& guid,
                             const Eigen::Vector<float, RW_EFF_CNT>& wheelSpeeds,
                             const std::array<bool, RW_EFF_CNT>& wheelAvailability);

   private:
    MrpFeedbackConfig cfg;
    uint64_t priorTime{};         //!< [ns]      Last time the attitude control is called
    Eigen::Vector3f int_sigma{};  //!< [s] integral of the MPR attitude error
};

#endif
