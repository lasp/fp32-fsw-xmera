#include "thrustVectoringAlgorithm_c.h"
#include "thrustVectoringAlgorithm.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/opaqueHandle.h"

#include <Eigen/Core>

bool ThrustVectoringAlgorithm_validateConfig(const Vector3f_c* r_MB_B, float thrust, const Vector3f_c* r_CB_B) {
    try {
        (void)ThrustVectoringConfig::create(
            cArrayToEigenVector3<float>(r_MB_B->data), thrust, cArrayToEigenVector3<float>(r_CB_B->data));
        return true;
    } catch (const fsw::invalid_argument&) {
        return false;
    }
}

ThrustVectoringAlgorithmHandle* ThrustVectoringAlgorithm_create(const Vector3f_c* r_MB_B,
                                                                float thrust,
                                                                const Vector3f_c* r_CB_B) {
    return fsw::createHandle<::ThrustVectoringAlgorithm, ThrustVectoringAlgorithmHandle>(ThrustVectoringConfig::create(
        cArrayToEigenVector3<float>(r_MB_B->data), thrust, cArrayToEigenVector3<float>(r_CB_B->data)));
}

void ThrustVectoringAlgorithm_destroy(ThrustVectoringAlgorithmHandle* self) {
    fsw::deleteHandle<::ThrustVectoringAlgorithm>(self);
}

void ThrustVectoringAlgorithm_setConfig(ThrustVectoringAlgorithmHandle* self,
                                        const Vector3f_c* r_MB_B,
                                        float thrust,
                                        const Vector3f_c* r_CB_B) {
    fsw::fromHandle<::ThrustVectoringAlgorithm>(self)->setConfig(ThrustVectoringConfig::create(
        cArrayToEigenVector3<float>(r_MB_B->data), thrust, cArrayToEigenVector3<float>(r_CB_B->data)));
}

Vector3f_c ThrustVectoringAlgorithm_update(const ThrustVectoringAlgorithmHandle* self, const Vector3f_c* Lreq_B) {
    const Eigen::Vector3f tHat_B =
        fsw::fromHandle<const ::ThrustVectoringAlgorithm>(self)->update(cArrayToEigenVector3<float>(Lreq_B->data));

    Vector3f_c result{};
    eigenVectorToCArray(tHat_B, result.data);
    return result;
}
