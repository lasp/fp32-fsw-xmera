#include "dvAccumulationAlgorithm_c.h"
#include "dvAccumulationAlgorithm.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/opaqueHandle.h"

bool DvAccumulationAlgorithm_validateConfig(float controlPeriod, Vector3f_c accelBias_B) {
    return DvAccumulationConfig::isValidControlPeriod(controlPeriod) &&
           DvAccumulationConfig::isValidAccelBias(cArrayToEigenVector3(accelBias_B.data));
}

DvAccumulationAlgorithmHandle* DvAccumulationAlgorithm_create(float controlPeriod, Vector3f_c accelBias_B) {
    return fsw::createHandle<::DvAccumulationAlgorithm, DvAccumulationAlgorithmHandle>(
        DvAccumulationConfig::create(controlPeriod, cArrayToEigenVector3(accelBias_B.data)));
}

void DvAccumulationAlgorithm_destroy(DvAccumulationAlgorithmHandle* self) {
    fsw::deleteHandle<::DvAccumulationAlgorithm>(self);
}

void DvAccumulationAlgorithm_setConfig(DvAccumulationAlgorithmHandle* self,
                                       float controlPeriod,
                                       Vector3f_c accelBias_B) {
    fsw::fromHandle<::DvAccumulationAlgorithm>(self)->setConfig(
        DvAccumulationConfig::create(controlPeriod, cArrayToEigenVector3(accelBias_B.data)));
}

void DvAccumulationAlgorithm_reInitialize(DvAccumulationAlgorithmHandle* self) {
    fsw::fromHandle<::DvAccumulationAlgorithm>(self)->reInitialize();
}

Vector3f_c DvAccumulationAlgorithm_update(DvAccumulationAlgorithmHandle* self, Vector3f_c rDDotNoGravity_BN_B) {
    const Eigen::Vector3f vehAccumDV_B =
        fsw::fromHandle<::DvAccumulationAlgorithm>(self)->update(cArrayToEigenVector3(rDDotNoGravity_BN_B.data));

    Vector3f_c result{};
    eigenVectorToCArray(vehAccumDV_B, result.data);
    return result;
}
