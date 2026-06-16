#include "inertial3DAlgorithm_c.h"
#include "inertial3DAlgorithm.h"
#include "inertial3DTypes.h"
#include "utilities/fsw/eigenSupport.h"

#include <Eigen/Core>

namespace {
Inertial3DConfig configFromC(const Inertial3DConfig_c& c) {
    return Inertial3DConfig::create(cArrayToEigenVector3<float>(c.sigma_RN.data));
}
}  // namespace

Inertial3DAlgorithmHandle* Inertial3DAlgorithm_create(const Inertial3DConfig_c* config) {
    return reinterpret_cast<Inertial3DAlgorithmHandle*>(new ::Inertial3DAlgorithm(configFromC(*config)));
}

void Inertial3DAlgorithm_destroy(Inertial3DAlgorithmHandle* self) {
    delete reinterpret_cast<::Inertial3DAlgorithm*>(self);
}

void Inertial3DAlgorithm_setConfig(Inertial3DAlgorithmHandle* self, const Inertial3DConfig_c* config) {
    reinterpret_cast<::Inertial3DAlgorithm*>(self)->setConfig(configFromC(*config));
}

Vector3f_c Inertial3DAlgorithm_update(const Inertial3DAlgorithmHandle* self) {
    const Eigen::Vector3f sigma_RN = reinterpret_cast<const ::Inertial3DAlgorithm*>(self)->update();
    Vector3f_c out{};
    eigenVectorToCArray(sigma_RN, out.data);
    return out;
}
