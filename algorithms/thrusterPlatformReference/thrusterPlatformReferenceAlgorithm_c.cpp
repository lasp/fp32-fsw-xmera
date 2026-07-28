#include "thrusterPlatformReferenceAlgorithm_c.h"
#include "thrusterPlatformReferenceAlgorithm.h"
#include "thrusterPlatformReferenceTypes.h"
#include "utilities/fsw/eigenSupport.h"

#include <Eigen/Core>

namespace {
ThrusterPlatformReferenceRwArrayConfiguration rwArrayConfigFromC(
    const ThrusterPlatformReferenceRwArrayConfiguration_c& c) {
    ThrusterPlatformReferenceRwArrayConfiguration out{};
    out.numRW = c.numRW;
    out.GsMatrix_B = cArrayToEigenMatrix<float, 3, kMaxNumRw>(c.GsMatrix_B);
    out.JsList = cArrayToEigenVector(c.JsList);
    return out;
}

ThrusterPlatformReferenceConfig configFromC(const ThrusterPlatformReferenceConfig_c& c) {
    return ThrusterPlatformReferenceConfig::create(cArrayToEigenVector3<float>(c.sigma_MB.data),
                                                   cArrayToEigenVector3<float>(c.r_BM_M.data),
                                                   cArrayToEigenVector3<float>(c.r_FM_F.data),
                                                   c.K,
                                                   c.Ki,
                                                   c.controlPeriod,
                                                   c.theta1Max,
                                                   c.theta2Max,
                                                   c.momentumDumping,
                                                   rwArrayConfigFromC(c.rwConfig));
}

ThrusterPlatformReferenceInputs inputsFromC(const ThrusterPlatformReferenceInputs_c& c) {
    ThrusterPlatformReferenceInputs out{};
    out.r_CB_B = cArrayToEigenVector3<float>(c.r_CB_B.data);
    out.r_TF_F = cArrayToEigenVector3<float>(c.r_TF_F.data);
    out.tHat_F = cArrayToEigenVector3<float>(c.tHat_F.data);
    out.thrust = c.thrust;
    out.wheelSpeeds = cArrayToEigenVector(c.wheelSpeeds);
    return out;
}
}  // namespace

uint32_t ThrusterPlatformReferenceAlgorithm_getMaxNumRw(void) { return THRUSTER_PLATFORM_REFERENCE_MAX_NUM_RW; }

ThrusterPlatformReferenceAlgorithmHandle* ThrusterPlatformReferenceAlgorithm_create(
    const ThrusterPlatformReferenceConfig_c* config) {
    return reinterpret_cast<ThrusterPlatformReferenceAlgorithmHandle*>(
        new ::ThrusterPlatformReferenceAlgorithm(configFromC(*config)));
}

void ThrusterPlatformReferenceAlgorithm_destroy(ThrusterPlatformReferenceAlgorithmHandle* self) {
    delete reinterpret_cast<::ThrusterPlatformReferenceAlgorithm*>(self);
}

void ThrusterPlatformReferenceAlgorithm_setConfig(ThrusterPlatformReferenceAlgorithmHandle* self,
                                                  const ThrusterPlatformReferenceConfig_c* config) {
    reinterpret_cast<::ThrusterPlatformReferenceAlgorithm*>(self)->setConfig(configFromC(*config));
}

void ThrusterPlatformReferenceAlgorithm_reInitialize(ThrusterPlatformReferenceAlgorithmHandle* self) {
    reinterpret_cast<::ThrusterPlatformReferenceAlgorithm*>(self)->reInitialize();
}

ThrusterPlatformReferenceOutput_c ThrusterPlatformReferenceAlgorithm_update(
    ThrusterPlatformReferenceAlgorithmHandle* self,
    const ThrusterPlatformReferenceInputs_c* inputs) {
    const ThrusterPlatformReferenceOutput out =
        reinterpret_cast<::ThrusterPlatformReferenceAlgorithm*>(self)->update(inputsFromC(*inputs));

    ThrusterPlatformReferenceOutput_c result{};
    result.theta1 = out.theta1;
    result.theta2 = out.theta2;
    eigenVectorToCArray(out.Lreq_B, result.Lreq_B.data);
    eigenVectorToCArray(out.r_TB_B, result.r_TB_B.data);
    eigenVectorToCArray(out.tHat_B, result.tHat_B.data);
    result.thrust = out.thrust;
    return result;
}
