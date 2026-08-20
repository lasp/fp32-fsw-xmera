#include "thrusterPlatformReferenceAlgorithm_c.h"
#include "thrusterPlatformReferenceAlgorithm.h"
#include "thrusterPlatformReferenceTypes.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/opaqueHandle.h"

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

ThrusterPlatformReferenceConfig makeConfig(const Vector3f_c& sigma_MB,
                                           const Vector3f_c& r_BM_M,
                                           const Vector3f_c& r_FM_F,
                                           float K,
                                           float Ki,
                                           float controlPeriod,
                                           float thetaMax,
                                           bool momentumDumping,
                                           const ThrusterPlatformReferenceRwArrayConfiguration_c& rwConfig) {
    return ThrusterPlatformReferenceConfig::create(cArrayToEigenVector3<float>(sigma_MB.data),
                                                   cArrayToEigenVector3<float>(r_BM_M.data),
                                                   cArrayToEigenVector3<float>(r_FM_F.data),
                                                   K,
                                                   Ki,
                                                   controlPeriod,
                                                   thetaMax,
                                                   momentumDumping,
                                                   rwArrayConfigFromC(rwConfig));
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

uint32_t ThrusterPlatformReferenceAlgorithm_getMaxNumRw(void) { return kMaxNumRw; }

bool ThrusterPlatformReferenceAlgorithm_validateConfig(
    const Vector3f_c* sigma_MB,
    const Vector3f_c* r_BM_M,
    const Vector3f_c* r_FM_F,
    float K,
    float Ki,
    float controlPeriod,
    float thetaMax,
    bool momentumDumping,
    const ThrusterPlatformReferenceRwArrayConfiguration_c* rwConfig) {
    try {
        (void)makeConfig(*sigma_MB, *r_BM_M, *r_FM_F, K, Ki, controlPeriod, thetaMax, momentumDumping, *rwConfig);
        return true;
    } catch (const fsw::invalid_argument&) {
        return false;
    }
}

ThrusterPlatformReferenceAlgorithmHandle* ThrusterPlatformReferenceAlgorithm_create(
    const Vector3f_c* sigma_MB,
    const Vector3f_c* r_BM_M,
    const Vector3f_c* r_FM_F,
    float K,
    float Ki,
    float controlPeriod,
    float thetaMax,
    bool momentumDumping,
    const ThrusterPlatformReferenceRwArrayConfiguration_c* rwConfig) {
    return reinterpret_cast<ThrusterPlatformReferenceAlgorithmHandle*>(new ::ThrusterPlatformReferenceAlgorithm(
        makeConfig(*sigma_MB, *r_BM_M, *r_FM_F, K, Ki, controlPeriod, thetaMax, momentumDumping, *rwConfig)));
}

void ThrusterPlatformReferenceAlgorithm_destroy(ThrusterPlatformReferenceAlgorithmHandle* self) {
    fsw::deleteHandle<::ThrusterPlatformReferenceAlgorithm>(self);
}

void ThrusterPlatformReferenceAlgorithm_setConfig(ThrusterPlatformReferenceAlgorithmHandle* self,
                                                  const Vector3f_c* sigma_MB,
                                                  const Vector3f_c* r_BM_M,
                                                  const Vector3f_c* r_FM_F,
                                                  float K,
                                                  float Ki,
                                                  float controlPeriod,
                                                  float thetaMax,
                                                  bool momentumDumping,
                                                  const ThrusterPlatformReferenceRwArrayConfiguration_c* rwConfig) {
    fsw::fromHandle<::ThrusterPlatformReferenceAlgorithm>(self)->setConfig(
        makeConfig(*sigma_MB, *r_BM_M, *r_FM_F, K, Ki, controlPeriod, thetaMax, momentumDumping, *rwConfig));
}

void ThrusterPlatformReferenceAlgorithm_reInitialize(ThrusterPlatformReferenceAlgorithmHandle* self) {
    fsw::fromHandle<::ThrusterPlatformReferenceAlgorithm>(self)->reInitialize();
}

ThrusterPlatformReferenceOutput_c ThrusterPlatformReferenceAlgorithm_update(
    ThrusterPlatformReferenceAlgorithmHandle* self,
    const ThrusterPlatformReferenceInputs_c* inputs) {
    const ThrusterPlatformReferenceOutput out =
        fsw::fromHandle<::ThrusterPlatformReferenceAlgorithm>(self)->update(inputsFromC(*inputs));

    ThrusterPlatformReferenceOutput_c result{};
    eigenVectorToCArray(out.Lcomp_B, result.Lcomp_B.data);
    eigenVectorToCArray(out.r_TB_B, result.r_TB_B.data);
    eigenVectorToCArray(out.tHat_B, result.tHat_B.data);
    result.thrust = out.thrust;
    return result;
}
