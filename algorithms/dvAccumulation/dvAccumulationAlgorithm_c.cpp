#include "dvAccumulationAlgorithm_c.h"
#include "dvAccumulationAlgorithm.h"
#include "msgPayloadDef/AccDataMsgF32Payload.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/opaqueHandle.h"

uint32_t DvAccumulationAlgorithm_getMaxAccBufPkt(void) { return MAX_ACC_BUF_PKT; }

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

DvAccumulationOutput_c DvAccumulationAlgorithm_update(DvAccumulationAlgorithmHandle* self,
                                                      const AccDataMsgF32Payload* accData) {
    const DvAccumulationOutput out = fsw::fromHandle<::DvAccumulationAlgorithm>(self)->update(*accData);

    DvAccumulationOutput_c result{};
    result.timeTag = out.timeTag;
    eigenVectorToCArray(out.vehAccumDV_B, result.vehAccumDV_B.data);
    return result;
}
