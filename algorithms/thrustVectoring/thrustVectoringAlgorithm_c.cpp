#include "thrustVectoringAlgorithm_c.h"
#include "thrustVectoringAlgorithm.h"
#include "thrustVectoringTypes.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/opaqueHandle.h"

#include <Eigen/Core>

namespace {
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) -- the vectors are distinct by frame and documented.
ThrustVectoringConfig makeConfig(const Vector3f_c& sigma_MB,
                                 const Vector3f_c& r_MB_B,
                                 const Vector3f_c& r_FM_F,
                                 float thetaMax,
                                 const Vector3f_c& r_TF_F,
                                 const Vector3f_c& tHat_F,
                                 float thrust,
                                 const Vector3f_c& r_CB_B) {
    const ThrustVectoringPlatformConfiguration platformConfig{.sigma_MB = cArrayToEigenVector3<float>(sigma_MB.data),
                                                              .r_MB_B = cArrayToEigenVector3<float>(r_MB_B.data),
                                                              .r_FM_F = cArrayToEigenVector3<float>(r_FM_F.data),
                                                              .thetaMax = thetaMax};
    const ThrustVectoringThrusterConfiguration thrusterConfig{.r_TF_F = cArrayToEigenVector3<float>(r_TF_F.data),
                                                              .tHat_F = cArrayToEigenVector3<float>(tHat_F.data),
                                                              .thrust = thrust};
    return ThrustVectoringConfig::create(platformConfig, thrusterConfig, cArrayToEigenVector3<float>(r_CB_B.data));
}
}  // namespace

bool ThrustVectoringAlgorithm_validateConfig(const Vector3f_c* sigma_MB,
                                             const Vector3f_c* r_MB_B,
                                             const Vector3f_c* r_FM_F,
                                             float thetaMax,
                                             const Vector3f_c* r_TF_F,
                                             const Vector3f_c* tHat_F,
                                             float thrust,
                                             const Vector3f_c* r_CB_B) {
    try {
        (void)makeConfig(*sigma_MB, *r_MB_B, *r_FM_F, thetaMax, *r_TF_F, *tHat_F, thrust, *r_CB_B);
        return true;
    } catch (const fsw::invalid_argument&) {
        return false;
    }
}

ThrustVectoringAlgorithmHandle* ThrustVectoringAlgorithm_create(const Vector3f_c* sigma_MB,
                                                                const Vector3f_c* r_MB_B,
                                                                const Vector3f_c* r_FM_F,
                                                                float thetaMax,
                                                                const Vector3f_c* r_TF_F,
                                                                const Vector3f_c* tHat_F,
                                                                float thrust,
                                                                const Vector3f_c* r_CB_B) {
    return reinterpret_cast<ThrustVectoringAlgorithmHandle*>(new ::ThrustVectoringAlgorithm(
        makeConfig(*sigma_MB, *r_MB_B, *r_FM_F, thetaMax, *r_TF_F, *tHat_F, thrust, *r_CB_B)));
}

void ThrustVectoringAlgorithm_destroy(ThrustVectoringAlgorithmHandle* self) {
    fsw::deleteHandle<::ThrustVectoringAlgorithm>(self);
}

void ThrustVectoringAlgorithm_setConfig(ThrustVectoringAlgorithmHandle* self,
                                        const Vector3f_c* sigma_MB,
                                        const Vector3f_c* r_MB_B,
                                        const Vector3f_c* r_FM_F,
                                        float thetaMax,
                                        const Vector3f_c* r_TF_F,
                                        const Vector3f_c* tHat_F,
                                        float thrust,
                                        const Vector3f_c* r_CB_B) {
    fsw::fromHandle<::ThrustVectoringAlgorithm>(self)->setConfig(
        makeConfig(*sigma_MB, *r_MB_B, *r_FM_F, thetaMax, *r_TF_F, *tHat_F, thrust, *r_CB_B));
}

void ThrustVectoringAlgorithm_reInitialize(ThrustVectoringAlgorithmHandle* self) {
    fsw::fromHandle<::ThrustVectoringAlgorithm>(self)->reInitialize();
}

ThrustVectoringOutput_c ThrustVectoringAlgorithm_update(ThrustVectoringAlgorithmHandle* self,
                                                        const Vector3f_c* Lreq_B) {
    const ThrustVectoringOutput out =
        fsw::fromHandle<::ThrustVectoringAlgorithm>(self)->update(cArrayToEigenVector3<float>(Lreq_B->data));

    ThrustVectoringOutput_c result{};
    eigenVectorToCArray(out.r_TB_B, result.r_TB_B.data);
    eigenVectorToCArray(out.tHat_B, result.tHat_B.data);
    result.thrust = out.thrust;
    return result;
}
