#include "mrpFeedbackAlgorithm_c.h"
#include "mrpFeedbackAlgorithm.h"
#include "mrpFeedbackTypes.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/opaqueHandle.h"

#include <Eigen/Core>
#include <optional>

// The C-boundary RW count must match the system-wide RW_EFF_CNT, otherwise the fixed-size C arrays and the
// Eigen conversions below would disagree on the wheel count.
static_assert(MrpFeedbackConfig::kMaxNumRw == MRP_FEEDBACK_MAX_NUM_RW, "MRP_FEEDBACK_MAX_NUM_RW must match RW_EFF_CNT");

namespace {
MrpFeedbackInputRwData rwConfigFromC(const MrpFeedbackRwConfig_c& c) {
    MrpFeedbackInputRwData out{};
    out.numRW = c.numRW;
    out.GsMatrix_B = cArrayToEigenMatrix<float, 3, MrpFeedbackConfig::kMaxNumRw>(c.GsMatrix_B);
    for (uint32_t i = 0U; i < MrpFeedbackConfig::kMaxNumRw; ++i) {
        out.JsList[i] = c.JsList[i];
        out.wheelAvailability[i] = c.wheelAvailability[i];
    }
    return out;
}

MrpFeedbackConfig makeConfig(float K,
                             float P,
                             float Ki,
                             float integralLimit,
                             ControlLawType_c controlLawType,
                             float controlPeriod,
                             const Vector3f_c& knownTorquePntB_B,
                             const Matrix3f_c& ISCPntB_B,
                             const MrpFeedbackRwConfig_c* rwConfiguration) {
    const MrpFeedbackControlParameters controlParameters{
        .K = K,
        .P = P,
        .Ki = Ki,
        .integralLimit = integralLimit,
        .controlLawType = static_cast<ControlLawType>(controlLawType),
        .controlPeriod = controlPeriod,
    };

    std::optional<MrpFeedbackInputRwData> rwConfigurationData;
    if (rwConfiguration != nullptr) {
        rwConfigurationData = rwConfigFromC(*rwConfiguration);
    }

    return MrpFeedbackConfig::create(controlParameters,
                                     cArrayToEigenVector3<float>(knownTorquePntB_B.data),
                                     c2DArrayToEigenMatrix3(ISCPntB_B.data),
                                     rwConfigurationData);
}
}  // namespace

uint32_t MrpFeedbackAlgorithm_getMaxNumRw(void) { return MRP_FEEDBACK_MAX_NUM_RW; }

bool MrpFeedbackAlgorithm_validateConfig(float K,
                                         float P,
                                         float Ki,
                                         float integralLimit,
                                         ControlLawType_c controlLawType,
                                         float controlPeriod,
                                         const Vector3f_c* knownTorquePntB_B,
                                         const Matrix3f_c* ISCPntB_B,
                                         const MrpFeedbackRwConfig_c* rwConfiguration) {
    try {
        (void)makeConfig(
            K, P, Ki, integralLimit, controlLawType, controlPeriod, *knownTorquePntB_B, *ISCPntB_B, rwConfiguration);
        return true;
    } catch (const fsw::invalid_argument&) {
        return false;
    }
}

MrpFeedbackAlgorithmHandle* MrpFeedbackAlgorithm_create(float K,
                                                        float P,
                                                        float Ki,
                                                        float integralLimit,
                                                        ControlLawType_c controlLawType,
                                                        float controlPeriod,
                                                        const Vector3f_c* knownTorquePntB_B,
                                                        const Matrix3f_c* ISCPntB_B,
                                                        const MrpFeedbackRwConfig_c* rwConfiguration) {
    return fsw::createHandle<::MrpFeedbackAlgorithm, MrpFeedbackAlgorithmHandle>(makeConfig(
        K, P, Ki, integralLimit, controlLawType, controlPeriod, *knownTorquePntB_B, *ISCPntB_B, rwConfiguration));
}

void MrpFeedbackAlgorithm_destroy(MrpFeedbackAlgorithmHandle* self) { fsw::deleteHandle<::MrpFeedbackAlgorithm>(self); }

void MrpFeedbackAlgorithm_setConfig(MrpFeedbackAlgorithmHandle* self,
                                    float K,
                                    float P,
                                    float Ki,
                                    float integralLimit,
                                    ControlLawType_c controlLawType,
                                    float controlPeriod,
                                    const Vector3f_c* knownTorquePntB_B,
                                    const Matrix3f_c* ISCPntB_B,
                                    const MrpFeedbackRwConfig_c* rwConfiguration) {
    fsw::fromHandle<::MrpFeedbackAlgorithm>(self)->setConfig(makeConfig(
        K, P, Ki, integralLimit, controlLawType, controlPeriod, *knownTorquePntB_B, *ISCPntB_B, rwConfiguration));
}

void MrpFeedbackAlgorithm_reInitialize(MrpFeedbackAlgorithmHandle* self) {
    fsw::fromHandle<::MrpFeedbackAlgorithm>(self)->reInitialize();
}

MrpFeedbackOutput_c MrpFeedbackAlgorithm_update(MrpFeedbackAlgorithmHandle* self,
                                                const MrpFeedbackInputGuidance_c* attGuidInput,
                                                const MrpFeedbackRwSpeeds_c* wheelSpeeds) {
    MrpFeedbackInputGuidance attGuidInputData{};
    attGuidInputData.sigma_BR = cArrayToEigenVector3<float>(attGuidInput->sigma_BR.data);
    attGuidInputData.omega_BR_B = cArrayToEigenVector3<float>(attGuidInput->omega_BR_B.data);
    attGuidInputData.omega_RN_B = cArrayToEigenVector3<float>(attGuidInput->omega_RN_B.data);
    attGuidInputData.domega_RN_B = cArrayToEigenVector3<float>(attGuidInput->domega_RN_B.data);

    std::array<float, RW_EFF_CNT> speeds{};
    for (uint32_t i = 0U; i < MrpFeedbackConfig::kMaxNumRw; ++i) {
        speeds[i] = wheelSpeeds->wheelSpeeds[i];
    }

    const MrpFeedbackOutput out = fsw::fromHandle<::MrpFeedbackAlgorithm>(self)->update(attGuidInputData, speeds);

    MrpFeedbackOutput_c result{};
    eigenVectorToCArray(out.controlTorque, result.controlTorque.data);
    eigenVectorToCArray(out.integralFeedbackTorque, result.integralFeedbackTorque.data);
    return result;
}
