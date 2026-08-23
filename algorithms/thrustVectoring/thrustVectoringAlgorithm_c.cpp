#include "thrustVectoringAlgorithm_c.h"
#include "thrustVectoringAlgorithm.h"
#include "thrustVectoringTypes.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/opaqueHandle.h"

#include <Eigen/Core>

namespace {
ThrustVectoringConfig makeConfig(const Vector3f_c& sigma_MB,
                                 const Vector3f_c& r_MB_B,
                                 const Vector3f_c& r_FM_F,
                                 float thetaMax) {
    return ThrustVectoringConfig::create(cArrayToEigenVector3<float>(sigma_MB.data),
                                         cArrayToEigenVector3<float>(r_MB_B.data),
                                         cArrayToEigenVector3<float>(r_FM_F.data),
                                         thetaMax);
}

ThrustVectoringInputs inputsFromC(const ThrustVectoringInputs_c& c) {
    ThrustVectoringInputs out{};
    out.r_CB_B = cArrayToEigenVector3<float>(c.r_CB_B.data);
    out.r_TF_F = cArrayToEigenVector3<float>(c.r_TF_F.data);
    out.tHat_F = cArrayToEigenVector3<float>(c.tHat_F.data);
    out.thrust = c.thrust;
    out.Lreq_B = cArrayToEigenVector3<float>(c.Lreq_B.data);
    return out;
}
}  // namespace

bool ThrustVectoringAlgorithm_validateConfig(const Vector3f_c* sigma_MB,
                                             const Vector3f_c* r_MB_B,
                                             const Vector3f_c* r_FM_F,
                                             float thetaMax) {
    try {
        (void)makeConfig(*sigma_MB, *r_MB_B, *r_FM_F, thetaMax);
        return true;
    } catch (const fsw::invalid_argument&) {
        return false;
    }
}

ThrustVectoringAlgorithmHandle* ThrustVectoringAlgorithm_create(const Vector3f_c* sigma_MB,
                                                                const Vector3f_c* r_MB_B,
                                                                const Vector3f_c* r_FM_F,
                                                                float thetaMax) {
    return reinterpret_cast<ThrustVectoringAlgorithmHandle*>(
        new ::ThrustVectoringAlgorithm(makeConfig(*sigma_MB, *r_MB_B, *r_FM_F, thetaMax)));
}

void ThrustVectoringAlgorithm_destroy(ThrustVectoringAlgorithmHandle* self) {
    fsw::deleteHandle<::ThrustVectoringAlgorithm>(self);
}

void ThrustVectoringAlgorithm_setConfig(ThrustVectoringAlgorithmHandle* self,
                                        const Vector3f_c* sigma_MB,
                                        const Vector3f_c* r_MB_B,
                                        const Vector3f_c* r_FM_F,
                                        float thetaMax) {
    fsw::fromHandle<::ThrustVectoringAlgorithm>(self)->setConfig(makeConfig(*sigma_MB, *r_MB_B, *r_FM_F, thetaMax));
}

void ThrustVectoringAlgorithm_reInitialize(ThrustVectoringAlgorithmHandle* self) {
    fsw::fromHandle<::ThrustVectoringAlgorithm>(self)->reInitialize();
}

ThrustVectoringOutput_c ThrustVectoringAlgorithm_update(ThrustVectoringAlgorithmHandle* self,
                                                        const ThrustVectoringInputs_c* inputs) {
    const ThrustVectoringOutput out = fsw::fromHandle<::ThrustVectoringAlgorithm>(self)->update(inputsFromC(*inputs));

    ThrustVectoringOutput_c result{};
    eigenVectorToCArray(out.r_TB_B, result.r_TB_B.data);
    eigenVectorToCArray(out.tHat_B, result.tHat_B.data);
    result.thrust = out.thrust;
    return result;
}
