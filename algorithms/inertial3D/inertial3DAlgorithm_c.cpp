#include "inertial3DAlgorithm_c.h"
#include "inertial3DAlgorithm.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/opaqueHandle.h"

#include <Eigen/Core>

namespace {
Inertial3DConfig configFromC(const Vector3f_c& sigma_RN) {
    return Inertial3DConfig::create(cArrayToEigenVector3<float>(sigma_RN.data));
}
}  // namespace

Inertial3DAlgorithmHandle* Inertial3DAlgorithm_create(const Vector3f_c sigma_RN) {
    return fsw::createHandle<::Inertial3DAlgorithm, Inertial3DAlgorithmHandle>(configFromC(sigma_RN));
}

void Inertial3DAlgorithm_destroy(Inertial3DAlgorithmHandle* self) { fsw::deleteHandle<::Inertial3DAlgorithm>(self); }

void Inertial3DAlgorithm_setConfig(Inertial3DAlgorithmHandle* self, const Vector3f_c sigma_RN) {
    fsw::fromHandle<::Inertial3DAlgorithm>(self)->setConfig(configFromC(sigma_RN));
}

Vector3f_c Inertial3DAlgorithm_update(const Inertial3DAlgorithmHandle* self) {
    const Eigen::Vector3f sigma_RN = fsw::fromHandle<const ::Inertial3DAlgorithm>(self)->update();
    Vector3f_c out{};
    eigenVectorToCArray(sigma_RN, out.data);
    return out;
}
