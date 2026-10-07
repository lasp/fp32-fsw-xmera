#include "vectorDutyCycleAlgorithm_c.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/opaqueHandle.h"
#include "vectorDutyCycleAlgorithm.h"

#include <Eigen/Core>

bool VectorDutyCycleAlgorithm_validateConfig(uint32_t onPeriods, uint32_t offPeriods) {
    try {
        (void)VectorDutyCycleConfig::create(onPeriods, offPeriods);
        return true;
    } catch (const fsw::invalid_argument&) {
        return false;
    }
}

VectorDutyCycleAlgorithmHandle* VectorDutyCycleAlgorithm_create(uint32_t onPeriods, uint32_t offPeriods) {
    return fsw::createHandle<::VectorDutyCycleAlgorithm, VectorDutyCycleAlgorithmHandle>(
        VectorDutyCycleConfig::create(onPeriods, offPeriods));
}

void VectorDutyCycleAlgorithm_destroy(VectorDutyCycleAlgorithmHandle* self) {
    fsw::deleteHandle<::VectorDutyCycleAlgorithm>(self);
}

void VectorDutyCycleAlgorithm_setConfig(VectorDutyCycleAlgorithmHandle* self, uint32_t onPeriods, uint32_t offPeriods) {
    fsw::fromHandle<::VectorDutyCycleAlgorithm>(self)->setConfig(VectorDutyCycleConfig::create(onPeriods, offPeriods));
}

void VectorDutyCycleAlgorithm_reInitialize(VectorDutyCycleAlgorithmHandle* self) {
    fsw::fromHandle<::VectorDutyCycleAlgorithm>(self)->reInitialize();
}

Vector3f_c VectorDutyCycleAlgorithm_update(VectorDutyCycleAlgorithmHandle* self, const Vector3f_c* inputVector) {
    const Eigen::Vector3f outputVector =
        fsw::fromHandle<::VectorDutyCycleAlgorithm>(self)->update(cArrayToEigenVector3<float>(inputVector->data));

    Vector3f_c out{};
    eigenVectorToCArray(outputVector, out.data);

    return out;
}
