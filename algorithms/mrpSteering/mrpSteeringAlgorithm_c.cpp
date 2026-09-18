#include "mrpSteeringAlgorithm_c.h"
#include "mrpSteeringAlgorithm.h"
#include "mrpSteeringTypes.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/opaqueHandle.h"

#include <Eigen/Core>
#include <optional>

namespace {
InputRwData rwConfigFromC(const MrpSteeringRwConfig_c& c) {
    InputRwData out{};
    out.GsMatrix_B = cArrayToEigenMatrix<float, 3, kMaxNumRw>(c.GsMatrix_B);
    for (uint32_t i = 0U; i < kMaxNumRw; ++i) {
        out.JsList[i] = c.JsList[i];
        out.wheelAvailability[i] = fsw::toDeviceAvailability(c.wheelAvailability[i]);
    }
    return out;
}

MrpSteeringConfig configFromC(const float K1,
                              const float K3,
                              const float omegaMax,
                              const bool ignoreOuterLoopFeedforward,
                              const float P,
                              const float Ki,
                              const float integralLimit,
                              const float controlPeriod,
                              const Vector3f_c& knownTorquePntB_B,
                              const Matrix3f_c& ISCPntB_B,
                              const MrpSteeringRwConfig_c* rwConfigurationC) {
    const MrpSteeringControlParameters controlParameters{
        .K1 = K1,
        .K3 = K3,
        .omegaMax = omegaMax,
        .ignoreOuterLoopFeedforward = ignoreOuterLoopFeedforward,
        .P = P,
        .Ki = Ki,
        .integralLimit = integralLimit,
        .controlPeriod = controlPeriod,
    };

    std::optional<InputRwData> rwConfiguration;
    if (rwConfigurationC != nullptr) {
        rwConfiguration = rwConfigFromC(*rwConfigurationC);
    }

    return MrpSteeringConfig::create(controlParameters,
                                     cArrayToEigenVector3<float>(knownTorquePntB_B.data),
                                     c2DArrayToEigenMatrix3(ISCPntB_B.data),
                                     rwConfiguration);
}
}  // namespace

uint32_t MrpSteeringAlgorithm_getMaxNumRw(void) { return kMaxNumRw; }

MrpSteeringAlgorithmHandle* MrpSteeringAlgorithm_create(float K1,
                                                        float K3,
                                                        float omegaMax,
                                                        bool ignoreOuterLoopFeedforward,
                                                        float P,
                                                        float Ki,
                                                        float integralLimit,
                                                        float controlPeriod,
                                                        const Vector3f_c* knownTorquePntB_B,
                                                        const Matrix3f_c* ISCPntB_B,
                                                        const MrpSteeringRwConfig_c* rwConfiguration) {
    return fsw::createHandle<::MrpSteeringAlgorithm, MrpSteeringAlgorithmHandle>(configFromC(K1,
                                                                                             K3,
                                                                                             omegaMax,
                                                                                             ignoreOuterLoopFeedforward,
                                                                                             P,
                                                                                             Ki,
                                                                                             integralLimit,
                                                                                             controlPeriod,
                                                                                             *knownTorquePntB_B,
                                                                                             *ISCPntB_B,
                                                                                             rwConfiguration));
}

void MrpSteeringAlgorithm_destroy(MrpSteeringAlgorithmHandle* self) { fsw::deleteHandle<::MrpSteeringAlgorithm>(self); }

void MrpSteeringAlgorithm_setConfig(MrpSteeringAlgorithmHandle* self,
                                    float K1,
                                    float K3,
                                    float omegaMax,
                                    bool ignoreOuterLoopFeedforward,
                                    float P,
                                    float Ki,
                                    float integralLimit,
                                    float controlPeriod,
                                    const Vector3f_c* knownTorquePntB_B,
                                    const Matrix3f_c* ISCPntB_B,
                                    const MrpSteeringRwConfig_c* rwConfiguration) {
    fsw::fromHandle<::MrpSteeringAlgorithm>(self)->setConfig(configFromC(K1,
                                                                         K3,
                                                                         omegaMax,
                                                                         ignoreOuterLoopFeedforward,
                                                                         P,
                                                                         Ki,
                                                                         integralLimit,
                                                                         controlPeriod,
                                                                         *knownTorquePntB_B,
                                                                         *ISCPntB_B,
                                                                         rwConfiguration));
}

void MrpSteeringAlgorithm_reInitialize(MrpSteeringAlgorithmHandle* self) {
    fsw::fromHandle<::MrpSteeringAlgorithm>(self)->reInitialize();
}

Vector3f_c MrpSteeringAlgorithm_update(MrpSteeringAlgorithmHandle* self,
                                       const MrpSteeringInputGuidance_c* attGuidInput,
                                       const MrpSteeringRwSpeeds_c* wheelSpeeds) {
    InputGuidanceData attGuidInputData{};
    attGuidInputData.sigma_BR = cArrayToEigenVector3<float>(attGuidInput->sigma_BR.data);
    attGuidInputData.omega_BR_B = cArrayToEigenVector3<float>(attGuidInput->omega_BR_B.data);
    attGuidInputData.omega_RN_B = cArrayToEigenVector3<float>(attGuidInput->omega_RN_B.data);
    attGuidInputData.domega_RN_B = cArrayToEigenVector3<float>(attGuidInput->domega_RN_B.data);

    std::array<float, kMaxNumRw> speeds{};
    for (uint32_t i = 0U; i < kMaxNumRw; ++i) {
        speeds[i] = wheelSpeeds->wheelSpeeds[i];
    }

    const Eigen::Vector3f torque = fsw::fromHandle<::MrpSteeringAlgorithm>(self)->update(attGuidInputData, speeds);

    Vector3f_c out{};
    eigenVectorToCArray(torque, out.data);
    return out;
}
