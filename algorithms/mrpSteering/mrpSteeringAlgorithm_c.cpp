#include "mrpSteeringAlgorithm_c.h"
#include "mrpSteeringAlgorithm.h"
#include "mrpSteeringTypes.h"
#include "utilities/fsw/eigenSupport.h"

#include <Eigen/Core>
#include <optional>

// The C-boundary RW count must match the system-wide RW_EFF_CNT, otherwise the fixed-size C arrays and the
// Eigen conversions below would disagree on the wheel count.
static_assert(kMaxNumRw == MRP_STEERING_MAX_NUM_RW, "MRP_STEERING_MAX_NUM_RW must match RW_EFF_CNT");

namespace {
InputRwData rwConfigFromC(const MrpSteeringRwConfig_c& c) {
    InputRwData out{};
    out.numRW = c.numRW;
    out.GsMatrix_B = cArrayToEigenMatrix<float, 3, kMaxNumRw>(c.GsMatrix_B);
    for (uint32_t i = 0U; i < kMaxNumRw; ++i) {
        out.JsList[i] = c.JsList[i];
        out.wheelAvailability[i] = c.wheelAvailability[i];
    }
    return out;
}

MrpSteeringConfig configFromC(const MrpSteeringConfig_c& c) {
    const MrpSteeringControlParameters controlParameters{
        .K1 = c.controlParameters.K1,
        .K3 = c.controlParameters.K3,
        .omegaMax = c.controlParameters.omegaMax,
        .ignoreOuterLoopFeedforward = c.controlParameters.ignoreOuterLoopFeedforward,
        .P = c.controlParameters.P,
        .Ki = c.controlParameters.Ki,
        .integralLimit = c.controlParameters.integralLimit,
        .controlPeriod = c.controlParameters.controlPeriod,
    };

    std::optional<InputRwData> rwConfiguration;
    if (c.hasRwConfiguration) {
        rwConfiguration = rwConfigFromC(c.rwConfiguration);
    }

    return MrpSteeringConfig::create(controlParameters,
                                     cArrayToEigenVector3<float>(c.knownTorquePntB_B.data),
                                     c2DArrayToEigenMatrix3(c.ISCPntB_B.data),
                                     rwConfiguration);
}
}  // namespace

uint32_t MrpSteeringAlgorithm_getMaxNumRw(void) { return MRP_STEERING_MAX_NUM_RW; }

MrpSteeringAlgorithmHandle* MrpSteeringAlgorithm_create(const MrpSteeringConfig_c* config) {
    return reinterpret_cast<MrpSteeringAlgorithmHandle*>(new ::MrpSteeringAlgorithm(configFromC(*config)));
}

void MrpSteeringAlgorithm_destroy(MrpSteeringAlgorithmHandle* self) {
    delete reinterpret_cast<::MrpSteeringAlgorithm*>(self);
}

void MrpSteeringAlgorithm_setConfig(MrpSteeringAlgorithmHandle* self, const MrpSteeringConfig_c* config) {
    reinterpret_cast<::MrpSteeringAlgorithm*>(self)->setConfig(configFromC(*config));
}

void MrpSteeringAlgorithm_reInitialize(MrpSteeringAlgorithmHandle* self) {
    reinterpret_cast<::MrpSteeringAlgorithm*>(self)->reInitialize();
}

Vector3f_c MrpSteeringAlgorithm_update(MrpSteeringAlgorithmHandle* self,
                                       const MrpSteeringInputGuidance_c* attGuidInput,
                                       const MrpSteeringRwSpeeds_c* wheelSpeeds) {
    InputGuidanceData attGuidInputData{};
    attGuidInputData.sigma_BR = cArrayToEigenVector3<float>(attGuidInput->sigma_BR.data);
    attGuidInputData.omega_BR_B = cArrayToEigenVector3<float>(attGuidInput->omega_BR_B.data);
    attGuidInputData.omega_RN_B = cArrayToEigenVector3<float>(attGuidInput->omega_RN_B.data);
    attGuidInputData.domega_RN_B = cArrayToEigenVector3<float>(attGuidInput->domega_RN_B.data);

    std::array<float, RW_EFF_CNT> speeds{};
    for (uint32_t i = 0U; i < kMaxNumRw; ++i) {
        speeds[i] = wheelSpeeds->wheelSpeeds[i];
    }

    const Eigen::Vector3f torque = reinterpret_cast<::MrpSteeringAlgorithm*>(self)->update(attGuidInputData, speeds);

    Vector3f_c out{};
    eigenVectorToCArray(torque, out.data);
    return out;
}
