#include "rateControlAlgorithm_c.h"

#include "rateControlAlgorithm.h"

#include <Eigen/Core>

RateControlAlgorithm* RateControlAlgorithm_create(void) {
    return reinterpret_cast<RateControlAlgorithm*>(new ::RateControlAlgorithm());
}

void RateControlAlgorithm_destroy(RateControlAlgorithm* self) {
    delete reinterpret_cast<::RateControlAlgorithm*>(self);
}

Eigen::Vector3f RateControlAlgorithm_update(const RateControlAlgorithm* self,
                                            const Eigen::Vector3f& omega_BR_B,
                                            const Eigen::Vector3f& domega_RN_B) {
    return reinterpret_cast<const ::RateControlAlgorithm*>(self)->update(omega_BR_B, domega_RN_B);
}

void RateControlAlgorithm_setSpacecraftInertia(RateControlAlgorithm* self, const Eigen::Matrix3f& spacecraftInertia) {
    reinterpret_cast<::RateControlAlgorithm*>(self)->setSpacecraftInertia(spacecraftInertia);
}

void RateControlAlgorithm_setDerivativeGainP(RateControlAlgorithm* self, float P) {
    reinterpret_cast<::RateControlAlgorithm*>(self)->setDerivativeGainP(P);
}

float RateControlAlgorithm_getDerivativeGainP(const RateControlAlgorithm* self) {
    return reinterpret_cast<const ::RateControlAlgorithm*>(self)->getDerivativeGainP();
}

void RateControlAlgorithm_setKnownTorquePntB_B(RateControlAlgorithm* self, Vector3f_c knownTorquePntB_B) {
    Eigen::Vector3f eigenVec;
    eigenVec << knownTorquePntB_B.data[0], knownTorquePntB_B.data[1], knownTorquePntB_B.data[2];
    reinterpret_cast<::RateControlAlgorithm*>(self)->setKnownTorquePntB_B(eigenVec);
}

Vector3f_c RateControlAlgorithm_getKnownTorquePntB_B(const RateControlAlgorithm* self) {
    const Eigen::Vector3f& eigenVec = reinterpret_cast<const ::RateControlAlgorithm*>(self)->getKnownTorquePntB_B();
    Vector3f_c out;
    out.data[0] = eigenVec[0];
    out.data[1] = eigenVec[1];
    out.data[2] = eigenVec[2];
    return out;
}
