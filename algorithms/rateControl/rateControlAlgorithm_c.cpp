#include "rateControlAlgorithm_c.h"
#include "architecture/utilities/eigenSupport.h"
#include "rateControlAlgorithm.h"

#include <Eigen/Core>

RateControlAlgorithm* RateControlAlgorithm_create(void) {
    return reinterpret_cast<RateControlAlgorithm*>(new ::RateControlAlgorithm());
}

void RateControlAlgorithm_destroy(RateControlAlgorithm* self) {
    delete reinterpret_cast<::RateControlAlgorithm*>(self);
}

Vector3f_c RateControlAlgorithm_update(const RateControlAlgorithm* self,
                                       const Vector3f_c& omega_BR_B,
                                       const Vector3f_c& domega_RN_B) {
    const Eigen::Vector3f vec_omega_BR_B = cArrayToEigenVector3(omega_BR_B.data);
    const Eigen::Vector3f vec_domega_RN_B = cArrayToEigenVector3(domega_RN_B.data);
    const Eigen::Vector3f vec =
        reinterpret_cast<const ::RateControlAlgorithm*>(self)->update(vec_omega_BR_B, vec_domega_RN_B);
    Vector3f_c out{};
    eigenVectorToCArray(vec, out.data);
    return out;
}

void RateControlAlgorithm_setSpacecraftInertia(RateControlAlgorithm* self, const Matrix3f_c& spacecraftInertia) {
    reinterpret_cast<::RateControlAlgorithm*>(self)->setSpacecraftInertia(
        c2DArrayToEigenMatrix3(spacecraftInertia.data));
}

void RateControlAlgorithm_setDerivativeGainP(RateControlAlgorithm* self, const float P) {
    reinterpret_cast<::RateControlAlgorithm*>(self)->setDerivativeGainP(P);
}

float RateControlAlgorithm_getDerivativeGainP(const RateControlAlgorithm* self) {
    return reinterpret_cast<const ::RateControlAlgorithm*>(self)->getDerivativeGainP();
}

void RateControlAlgorithm_setKnownTorquePntB_B(RateControlAlgorithm* self, const Vector3f_c knownTorquePntB_B) {
    const Eigen::Vector3f eigenVec = cArrayToEigenVector3(knownTorquePntB_B.data);
    reinterpret_cast<::RateControlAlgorithm*>(self)->setKnownTorquePntB_B(eigenVec);
}

Vector3f_c RateControlAlgorithm_getKnownTorquePntB_B(const RateControlAlgorithm* self) {
    const Eigen::Vector3f& eigenVec = reinterpret_cast<const ::RateControlAlgorithm*>(self)->getKnownTorquePntB_B();
    Vector3f_c out{};
    eigenVectorToCArray(eigenVec, out.data);
    return out;
}
