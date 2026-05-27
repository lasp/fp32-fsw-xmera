#include "inertial3DAlgorithm_c.h"
#include "inertial3DAlgorithm.h"

#include <Eigen/Core>

Inertial3DAlgorithmHandle* Inertial3DAlgorithm_create(void) {
    return reinterpret_cast<Inertial3DAlgorithmHandle*>(new ::Inertial3DAlgorithm());
}

void Inertial3DAlgorithm_destroy(Inertial3DAlgorithmHandle* self) {
    delete reinterpret_cast<::Inertial3DAlgorithm*>(self);
}

AttRefMsgF32Payload Inertial3DAlgorithm_update(const Inertial3DAlgorithmHandle* self) {
    return reinterpret_cast<const ::Inertial3DAlgorithm*>(self)->update();
}

void Inertial3DAlgorithm_setSigmaRN(Inertial3DAlgorithmHandle* self, Vector3f_c sigmaRN) {
    Eigen::Vector3f vec;
    vec << sigmaRN.data[0], sigmaRN.data[1], sigmaRN.data[2];
    reinterpret_cast<::Inertial3DAlgorithm*>(self)->setSigmaRN(vec);
}

Vector3f_c Inertial3DAlgorithm_getSigmaRN(const Inertial3DAlgorithmHandle* self) {
    Eigen::Vector3f vec = reinterpret_cast<const ::Inertial3DAlgorithm*>(self)->getSigmaRN();
    Vector3f_c out;
    out.data[0] = vec[0];
    out.data[1] = vec[1];
    out.data[2] = vec[2];
    return out;
}
