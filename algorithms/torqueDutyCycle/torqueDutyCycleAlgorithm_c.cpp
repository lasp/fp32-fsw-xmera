#include "torqueDutyCycleAlgorithm_c.h"
#include "torqueDutyCycleAlgorithm.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/opaqueHandle.h"

#include <Eigen/Core>

bool TorqueDutyCycleAlgorithm_validateConfig(uint32_t onPeriods, uint32_t offPeriods) {
    try {
        (void)TorqueDutyCycleConfig::create(onPeriods, offPeriods);
        return true;
    } catch (const fsw::invalid_argument&) {
        return false;
    }
}

TorqueDutyCycleAlgorithmHandle* TorqueDutyCycleAlgorithm_create(uint32_t onPeriods, uint32_t offPeriods) {
    return fsw::createHandle<::TorqueDutyCycleAlgorithm, TorqueDutyCycleAlgorithmHandle>(
        TorqueDutyCycleConfig::create(onPeriods, offPeriods));
}

void TorqueDutyCycleAlgorithm_destroy(TorqueDutyCycleAlgorithmHandle* self) {
    fsw::deleteHandle<::TorqueDutyCycleAlgorithm>(self);
}

void TorqueDutyCycleAlgorithm_setConfig(TorqueDutyCycleAlgorithmHandle* self, uint32_t onPeriods, uint32_t offPeriods) {
    fsw::fromHandle<::TorqueDutyCycleAlgorithm>(self)->setConfig(TorqueDutyCycleConfig::create(onPeriods, offPeriods));
}

void TorqueDutyCycleAlgorithm_reInitialize(TorqueDutyCycleAlgorithmHandle* self) {
    fsw::fromHandle<::TorqueDutyCycleAlgorithm>(self)->reInitialize();
}

Vector3f_c TorqueDutyCycleAlgorithm_update(TorqueDutyCycleAlgorithmHandle* self, const Vector3f_c* cmdTorque_B) {
    const Eigen::Vector3f gated_B =
        fsw::fromHandle<::TorqueDutyCycleAlgorithm>(self)->update(cArrayToEigenVector3<float>(cmdTorque_B->data));

    Vector3f_c out{};
    eigenVectorToCArray(gated_B, out.data);

    return out;
}
