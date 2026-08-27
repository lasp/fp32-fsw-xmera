#include "thrustVectoringAlgorithm_c.h"
#include "thrustVectoringAlgorithm.h"
#include "thrustVectoringTypes.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/opaqueHandle.h"

#include <Eigen/Core>

namespace {
ThrustVectoringConfig makeConfig(const Vector3f_c& sigma_MB,
                                 const Vector3f_c& r_MB_B,
                                 float thetaMax,
                                 float armLength,
                                 float thrust,
                                 const Vector3f_c& r_CB_B) {
    const ThrustVectoringPlatformConfiguration platformConfig{.sigma_MB = cArrayToEigenVector3<float>(sigma_MB.data),
                                                              .r_MB_B = cArrayToEigenVector3<float>(r_MB_B.data),
                                                              .thetaMax = thetaMax};
    const ThrustVectoringThrusterConfiguration thrusterConfig{.armLength = armLength, .thrust = thrust};
    return ThrustVectoringConfig::create(platformConfig, thrusterConfig, cArrayToEigenVector3<float>(r_CB_B.data));
}
}  // namespace

bool ThrustVectoringAlgorithm_validateConfig(const Vector3f_c* sigma_MB,
                                             const Vector3f_c* r_MB_B,
                                             float thetaMax,
                                             float armLength,
                                             float thrust,
                                             const Vector3f_c* r_CB_B) {
    try {
        (void)makeConfig(*sigma_MB, *r_MB_B, thetaMax, armLength, thrust, *r_CB_B);
        return true;
    } catch (const fsw::invalid_argument&) {
        return false;
    }
}

ThrustVectoringAlgorithmHandle* ThrustVectoringAlgorithm_create(const Vector3f_c* sigma_MB,
                                                                const Vector3f_c* r_MB_B,
                                                                float thetaMax,
                                                                float armLength,
                                                                float thrust,
                                                                const Vector3f_c* r_CB_B) {
    return fsw::createHandle<::ThrustVectoringAlgorithm, ThrustVectoringAlgorithmHandle>(
        makeConfig(*sigma_MB, *r_MB_B, thetaMax, armLength, thrust, *r_CB_B));
}

void ThrustVectoringAlgorithm_destroy(ThrustVectoringAlgorithmHandle* self) {
    fsw::deleteHandle<::ThrustVectoringAlgorithm>(self);
}

void ThrustVectoringAlgorithm_setConfig(ThrustVectoringAlgorithmHandle* self,
                                        const Vector3f_c* sigma_MB,
                                        const Vector3f_c* r_MB_B,
                                        float thetaMax,
                                        float armLength,
                                        float thrust,
                                        const Vector3f_c* r_CB_B) {
    fsw::fromHandle<::ThrustVectoringAlgorithm>(self)->setConfig(
        makeConfig(*sigma_MB, *r_MB_B, thetaMax, armLength, thrust, *r_CB_B));
}

ThrustVectoringOutput_c ThrustVectoringAlgorithm_update(const ThrustVectoringAlgorithmHandle* self,
                                                        const Vector3f_c* Lreq_B) {
    const ThrustVectoringOutput out =
        fsw::fromHandle<const ::ThrustVectoringAlgorithm>(self)->update(cArrayToEigenVector3<float>(Lreq_B->data));

    ThrustVectoringOutput_c result{};
    eigenVectorToCArray(out.r_TB_B, result.r_TB_B.data);
    eigenVectorToCArray(out.tHat_B, result.tHat_B.data);
    result.thrust = out.thrust;
    return result;
}
