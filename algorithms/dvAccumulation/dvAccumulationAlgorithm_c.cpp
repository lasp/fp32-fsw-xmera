#include "dvAccumulationAlgorithm_c.h"
#include "dvAccumulationAlgorithm.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/opaqueHandle.h"

DvAccumulationAlgorithmHandle* DvAccumulationAlgorithm_create(void) {
    return fsw::createHandle<::DvAccumulationAlgorithm, DvAccumulationAlgorithmHandle>();
}

void DvAccumulationAlgorithm_destroy(DvAccumulationAlgorithmHandle* self) {
    fsw::deleteHandle<::DvAccumulationAlgorithm>(self);
}

void DvAccumulationAlgorithm_reInitialize(DvAccumulationAlgorithmHandle* self) {
    fsw::fromHandle<::DvAccumulationAlgorithm>(self)->reInitialize();
}

void DvAccumulationAlgorithm_reInitializeExceptPersistentStates(DvAccumulationAlgorithmHandle* self) {
    fsw::fromHandle<::DvAccumulationAlgorithm>(self)->reInitializeExceptPersistentStates();
}

Vector3f_c DvAccumulationAlgorithm_update(DvAccumulationAlgorithmHandle* self,
                                          uint64_t callTime,
                                          Vector3f_c rDDotNoGravity_BN_B) {
    const Eigen::Vector3f accel_B = cArrayToEigenVector3(rDDotNoGravity_BN_B.data);
    const Eigen::Vector3f vehAccumDV_B = fsw::fromHandle<::DvAccumulationAlgorithm>(self)->update(callTime, accel_B);

    Vector3f_c result{};
    eigenVectorToCArray(vehAccumDV_B, result.data);
    return result;
}
