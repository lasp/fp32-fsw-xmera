#include "mrpFeedbackAlgorithm_c.h"
#include "mrpFeedbackAlgorithm.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/opaqueHandle.h"

#include <Eigen/Core>
#include <optional>

namespace {
MrpFeedbackInputRwData rwConfigFromC(const MrpFeedbackRwSpinAxes_c& GsMatrix_B,
                                     const MrpFeedbackRwInertias_c& JsList,
                                     const MrpFeedbackRwAvailability_c& wheelAvailability) {
    MrpFeedbackInputRwData out{};
    out.GsMatrix_B = cArrayToEigenMatrix<float, 3, kMaxNumRw>(GsMatrix_B.data);
    for (uint32_t i = 0U; i < kMaxNumRw; ++i) {
        out.JsList[i] = JsList.data[i];
        // An unscoped C enum can carry a value outside its enumerators. Converting through
        // toDeviceAvailability keeps any such value out.
        out.wheelAvailability[i] = fsw::toDeviceAvailability(wheelAvailability.availability[i]);
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
                             const MrpFeedbackRwSpinAxes_c* GsMatrix_B,
                             const MrpFeedbackRwInertias_c* JsList,
                             const MrpFeedbackRwAvailability_c* wheelAvailability) {
    const MrpFeedbackControlParameters controlParameters{
        .K = K,
        .P = P,
        .Ki = Ki,
        .integralLimit = integralLimit,
        .controlLawType = static_cast<ControlLawType>(controlLawType),
        .controlPeriod = controlPeriod,
    };

    // A null spin-axis array omits the reaction-wheel momentum term; the other two are then unused.
    std::optional<MrpFeedbackInputRwData> rwConfigurationData;
    if (GsMatrix_B != nullptr) {
        rwConfigurationData = rwConfigFromC(*GsMatrix_B, *JsList, *wheelAvailability);
    }

    return MrpFeedbackConfig::create(controlParameters,
                                     cArrayToEigenVector3<float>(knownTorquePntB_B.data),
                                     c2DArrayToEigenMatrix3(ISCPntB_B.data),
                                     rwConfigurationData);
}
}  // namespace

uint32_t MrpFeedbackAlgorithm_getMaxNumRw(void) { return kMaxNumRw; }

bool MrpFeedbackAlgorithm_validateConfig(float K,
                                         float P,
                                         float Ki,
                                         float integralLimit,
                                         ControlLawType_c controlLawType,
                                         float controlPeriod,
                                         const Vector3f_c* knownTorquePntB_B,
                                         const Matrix3f_c* ISCPntB_B,
                                         const MrpFeedbackRwSpinAxes_c* GsMatrix_B,
                                         const MrpFeedbackRwInertias_c* JsList,
                                         const MrpFeedbackRwAvailability_c* wheelAvailability) {
    try {
        (void)makeConfig(K,
                         P,
                         Ki,
                         integralLimit,
                         controlLawType,
                         controlPeriod,
                         *knownTorquePntB_B,
                         *ISCPntB_B,
                         GsMatrix_B,
                         JsList,
                         wheelAvailability);
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
                                                        const MrpFeedbackRwSpinAxes_c* GsMatrix_B,
                                                        const MrpFeedbackRwInertias_c* JsList,
                                                        const MrpFeedbackRwAvailability_c* wheelAvailability) {
    return fsw::createHandle<::MrpFeedbackAlgorithm, MrpFeedbackAlgorithmHandle>(makeConfig(K,
                                                                                            P,
                                                                                            Ki,
                                                                                            integralLimit,
                                                                                            controlLawType,
                                                                                            controlPeriod,
                                                                                            *knownTorquePntB_B,
                                                                                            *ISCPntB_B,
                                                                                            GsMatrix_B,
                                                                                            JsList,
                                                                                            wheelAvailability));
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
                                    const MrpFeedbackRwSpinAxes_c* GsMatrix_B,
                                    const MrpFeedbackRwInertias_c* JsList,
                                    const MrpFeedbackRwAvailability_c* wheelAvailability) {
    fsw::fromHandle<::MrpFeedbackAlgorithm>(self)->setConfig(makeConfig(K,
                                                                        P,
                                                                        Ki,
                                                                        integralLimit,
                                                                        controlLawType,
                                                                        controlPeriod,
                                                                        *knownTorquePntB_B,
                                                                        *ISCPntB_B,
                                                                        GsMatrix_B,
                                                                        JsList,
                                                                        wheelAvailability));
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

    std::array<float, kMaxNumRw> speeds{};
    for (uint32_t i = 0U; i < kMaxNumRw; ++i) {
        speeds[i] = wheelSpeeds->wheelSpeeds[i];
    }

    const MrpFeedbackOutput out = fsw::fromHandle<::MrpFeedbackAlgorithm>(self)->update(attGuidInputData, speeds);

    MrpFeedbackOutput_c result{};
    eigenVectorToCArray(out.controlTorque, result.controlTorque.data);
    eigenVectorToCArray(out.integralFeedbackTorque, result.integralFeedbackTorque.data);
    return result;
}
