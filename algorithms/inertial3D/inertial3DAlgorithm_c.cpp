/* MIT License
 *
 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#include "inertial3DAlgorithm_c.h"
#include "inertial3DAlgorithm.h"

#include <Eigen/Core>

Inertial3DAlgorithm* Inertial3DAlgorithm_create(void) {
    return reinterpret_cast<Inertial3DAlgorithm*>(new ::Inertial3DAlgorithm());
}

void Inertial3DAlgorithm_destroy(Inertial3DAlgorithm* self) { delete reinterpret_cast<::Inertial3DAlgorithm*>(self); }

AttRefMsgF32Payload Inertial3DAlgorithm_update(const Inertial3DAlgorithm* self) {
    return reinterpret_cast<const ::Inertial3DAlgorithm*>(self)->update();
}

void Inertial3DAlgorithm_setSigmaRN(Inertial3DAlgorithm* self, Vector3f_c sigmaRN) {
    Eigen::Vector3f vec;
    vec << sigmaRN.data[0], sigmaRN.data[1], sigmaRN.data[2];
    reinterpret_cast<::Inertial3DAlgorithm*>(self)->setSigmaRN(vec);
}

Vector3f_c Inertial3DAlgorithm_getSigmaRN(const Inertial3DAlgorithm* self) {
    Eigen::Vector3f vec = reinterpret_cast<const ::Inertial3DAlgorithm*>(self)->getSigmaRN();
    Vector3f_c out;
    out.data[0] = vec[0];
    out.data[1] = vec[1];
    out.data[2] = vec[2];
    return out;
}
