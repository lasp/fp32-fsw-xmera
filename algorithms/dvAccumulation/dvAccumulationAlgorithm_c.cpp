#include "dvAccumulationAlgorithm_c.h"
#include "dvAccumulationAlgorithm.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/opaqueHandle.h"

bool DvAccumulationAlgorithm_validateConfig(float controlPeriod) {
    return DvAccumulationConfig::isValidControlPeriod(controlPeriod);
}

DvAccumulationAlgorithmHandle* DvAccumulationAlgorithm_create(float controlPeriod) {
    return fsw::createHandle<::DvAccumulationAlgorithm, DvAccumulationAlgorithmHandle>(
        DvAccumulationConfig::create(controlPeriod));
}

void DvAccumulationAlgorithm_destroy(DvAccumulationAlgorithmHandle* self) {
    fsw::deleteHandle<::DvAccumulationAlgorithm>(self);
}

void DvAccumulationAlgorithm_setConfig(DvAccumulationAlgorithmHandle* self, float controlPeriod) {
    fsw::fromHandle<::DvAccumulationAlgorithm>(self)->setConfig(DvAccumulationConfig::create(controlPeriod));
}

void DvAccumulationAlgorithm_reInitialize(DvAccumulationAlgorithmHandle* self) {
    fsw::fromHandle<::DvAccumulationAlgorithm>(self)->reInitialize();
}

Vector3f_c DvAccumulationAlgorithm_update(DvAccumulationAlgorithmHandle* self, Vector3f_c rDDotNoGravity_BN_B) {
    const Eigen::Vector3f accel_B = cArrayToEigenVector3(rDDotNoGravity_BN_B.data);
    const Eigen::Vector3f vehAccumDV_B = fsw::fromHandle<::DvAccumulationAlgorithm>(self)->update(accel_B);

    Vector3f_c result{};
    eigenVectorToCArray(vehAccumDV_B, result.data);
    return result;
}
