#include "thrDesatDutyCycleAlgorithm_c.h"
#include "thrDesatDutyCycleAlgorithm.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/opaqueHandle.h"

#include <Eigen/Core>

bool ThrDesatDutyCycleAlgorithm_validateConfig(uint32_t firingPeriods, uint32_t settlingPeriods) {
    try {
        (void)ThrDesatDutyCycleConfig::create(firingPeriods, settlingPeriods);
        return true;
    } catch (const fsw::invalid_argument&) {
        return false;
    }
}

ThrDesatDutyCycleAlgorithmHandle* ThrDesatDutyCycleAlgorithm_create(uint32_t firingPeriods, uint32_t settlingPeriods) {
    return fsw::createHandle<::ThrDesatDutyCycleAlgorithm, ThrDesatDutyCycleAlgorithmHandle>(
        ThrDesatDutyCycleConfig::create(firingPeriods, settlingPeriods));
}

void ThrDesatDutyCycleAlgorithm_destroy(ThrDesatDutyCycleAlgorithmHandle* self) {
    fsw::deleteHandle<::ThrDesatDutyCycleAlgorithm>(self);
}

void ThrDesatDutyCycleAlgorithm_setConfig(ThrDesatDutyCycleAlgorithmHandle* self,
                                          uint32_t firingPeriods,
                                          uint32_t settlingPeriods) {
    fsw::fromHandle<::ThrDesatDutyCycleAlgorithm>(self)->setConfig(
        ThrDesatDutyCycleConfig::create(firingPeriods, settlingPeriods));
}

void ThrDesatDutyCycleAlgorithm_reInitialize(ThrDesatDutyCycleAlgorithmHandle* self) {
    fsw::fromHandle<::ThrDesatDutyCycleAlgorithm>(self)->reInitialize();
}

Vector3f_c ThrDesatDutyCycleAlgorithm_update(ThrDesatDutyCycleAlgorithmHandle* self, const Vector3f_c* cmdTorque_B) {
    const Eigen::Vector3f gated_B =
        fsw::fromHandle<::ThrDesatDutyCycleAlgorithm>(self)->update(cArrayToEigenVector3<float>(cmdTorque_B->data));

    Vector3f_c out{};
    eigenVectorToCArray(gated_B, out.data);

    return out;
}
