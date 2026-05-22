#include "dvAccumulationAlgorithm_c.h"
#include "architecture/utilities/eigenSupport.h"
#include "dvAccumulationAlgorithm.h"
#include "msgPayloadDef/AccDataMsgF32Payload.h"

uint32_t DvAccumulationAlgorithm_getMaxAccBufPkt(void) { return MAX_ACC_BUF_PKT; }

DvAccumulationAlgorithmHandle* DvAccumulationAlgorithm_create(void) {
    return reinterpret_cast<DvAccumulationAlgorithmHandle*>(
        new ::DvAccumulationAlgorithm(DvAccumulationConfig::create()));
}

void DvAccumulationAlgorithm_destroy(DvAccumulationAlgorithmHandle* self) {
    delete reinterpret_cast<::DvAccumulationAlgorithm*>(self);
}

void DvAccumulationAlgorithm_resetState(DvAccumulationAlgorithmHandle* self, const AccDataMsgF32Payload* accData) {
    reinterpret_cast<::DvAccumulationAlgorithm*>(self)->resetState(*accData);
}

DvAccumulationOutput_c DvAccumulationAlgorithm_update(DvAccumulationAlgorithmHandle* self,
                                                      const AccDataMsgF32Payload* accData) {
    const DvAccumulationOutput out = reinterpret_cast<::DvAccumulationAlgorithm*>(self)->update(*accData);

    DvAccumulationOutput_c result{};
    result.timeTag = out.timeTag;
    eigenVectorToCArray(out.vehAccumDV_B, result.vehAccumDV_B.data);
    return result;
}
