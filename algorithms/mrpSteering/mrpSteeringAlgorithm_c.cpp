#include "mrpSteeringAlgorithm_c.h"
#include "mrpSteeringAlgorithm.h"
#include "mrpSteeringTypes.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/opaqueHandle.h"

#include <Eigen/Core>
#include <optional>

namespace {
InputRwData rwConfigFromC(const MrpSteeringRwSpinAxes_c& GsMatrix_B,
                          const MrpSteeringRwInertias_c& JsList,
                          const MrpSteeringRwAvailability_c& wheelAvailability) {
    InputRwData out{};
    out.GsMatrix_B = cArrayToEigenMatrix<float, 3, kMaxNumRw>(GsMatrix_B.data);
    for (uint32_t i = 0U; i < kMaxNumRw; ++i) {
        out.JsList[i] = JsList.data[i];
        // An unscoped C enum can carry a value outside its enumerators. Converting through
        // toDeviceAvailability keeps any such value out.
        out.wheelAvailability[i] = fsw::toDeviceAvailability(wheelAvailability.availability[i]);
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
                              const MrpSteeringRwSpinAxes_c* GsMatrix_BC,
                              const MrpSteeringRwInertias_c* JsListC,
                              const MrpSteeringRwAvailability_c* wheelAvailabilityC) {
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
    // A null spin-axis array omits the reaction-wheel terms; the other two are then unused.
    if (GsMatrix_BC != nullptr) {
        rwConfiguration = rwConfigFromC(*GsMatrix_BC, *JsListC, *wheelAvailabilityC);
    }

    return MrpSteeringConfig::create(controlParameters,
                                     cArrayToEigenVector3<float>(knownTorquePntB_B.data),
                                     c2DArrayToEigenMatrix3(ISCPntB_B.data),
                                     rwConfiguration);
}
}  // namespace

uint32_t MrpSteeringAlgorithm_getMaxNumRw(void) { return kMaxNumRw; }

bool MrpSteeringAlgorithm_validateConfig(float K1,
                                         float K3,
                                         float omegaMax,
                                         bool ignoreOuterLoopFeedforward,
                                         float P,
                                         float Ki,
                                         float integralLimit,
                                         float controlPeriod,
                                         const Vector3f_c* knownTorquePntB_B,
                                         const Matrix3f_c* ISCPntB_B,
                                         const MrpSteeringRwSpinAxes_c* GsMatrix_B,
                                         const MrpSteeringRwInertias_c* JsList,
                                         const MrpSteeringRwAvailability_c* wheelAvailability) {
    // Build the config through the same path create() uses: success means valid, a throw means
    // invalid. Sharing configFromC keeps the predicate from drifting from what create() accepts.
    try {
        (void)configFromC(K1,
                          K3,
                          omegaMax,
                          ignoreOuterLoopFeedforward,
                          P,
                          Ki,
                          integralLimit,
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
                                                        const MrpSteeringRwSpinAxes_c* GsMatrix_B,
                                                        const MrpSteeringRwInertias_c* JsList,
                                                        const MrpSteeringRwAvailability_c* wheelAvailability) {
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
                                                                                             GsMatrix_B,
                                                                                             JsList,
                                                                                             wheelAvailability));
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
                                    const MrpSteeringRwSpinAxes_c* GsMatrix_B,
                                    const MrpSteeringRwInertias_c* JsList,
                                    const MrpSteeringRwAvailability_c* wheelAvailability) {
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
                                                                         GsMatrix_B,
                                                                         JsList,
                                                                         wheelAvailability));
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
