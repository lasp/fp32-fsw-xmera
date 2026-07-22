#include "dvExecuteGuidanceAlgorithm_c.h"
#include "dvExecuteGuidanceAlgorithm.h"
#include "dvExecuteGuidanceTypes.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/opaqueHandle.h"

#include <Eigen/Core>

namespace {
DvExecuteGuidanceOutput_c outputToC(const DvExecuteGuidanceOutput& out) {
    DvExecuteGuidanceOutput_c result{};
    result.burnExecuting = out.burnExecuting;
    result.burnComplete = out.burnComplete;
    result.commandThrustersOff = out.commandThrustersOff;
    return result;
}
}  // namespace

bool DvExecuteGuidanceAlgorithm_validateConfig(float minTime, float maxTime, float controlPeriod) {
    try {
        (void)DvExecuteGuidanceConfig::create(minTime, maxTime, controlPeriod);
        return true;
    } catch (const fsw::invalid_argument&) {
        return false;
    }
}

DvExecuteGuidanceAlgorithmHandle* DvExecuteGuidanceAlgorithm_create(float minTime, float maxTime, float controlPeriod) {
    return reinterpret_cast<DvExecuteGuidanceAlgorithmHandle*>(
        new ::DvExecuteGuidanceAlgorithm(DvExecuteGuidanceConfig::create(minTime, maxTime, controlPeriod)));
}

void DvExecuteGuidanceAlgorithm_destroy(DvExecuteGuidanceAlgorithmHandle* self) {
    fsw::deleteHandle<::DvExecuteGuidanceAlgorithm>(self);
}

void DvExecuteGuidanceAlgorithm_setConfig(DvExecuteGuidanceAlgorithmHandle* self,
                                          float minTime,
                                          float maxTime,
                                          float controlPeriod) {
    fsw::fromHandle<::DvExecuteGuidanceAlgorithm>(self)->setConfig(
        DvExecuteGuidanceConfig::create(minTime, maxTime, controlPeriod));
}

void DvExecuteGuidanceAlgorithm_reInitialize(DvExecuteGuidanceAlgorithmHandle* self) {
    fsw::fromHandle<::DvExecuteGuidanceAlgorithm>(self)->reInitialize();
}

DvExecuteGuidanceOutput_c DvExecuteGuidanceAlgorithm_update(DvExecuteGuidanceAlgorithmHandle* self,
                                                            uint64_t callTime,
                                                            const Vector3f_c* vehAccumDV,
                                                            const Vector3f_c* dvInrtlCmd,
                                                            uint64_t burnStartTime) {
    const DvExecuteGuidanceOutput out =
        fsw::fromHandle<::DvExecuteGuidanceAlgorithm>(self)->update(callTime,
                                                                    cArrayToEigenVector3<float>(vehAccumDV->data),
                                                                    cArrayToEigenVector3<float>(dvInrtlCmd->data),
                                                                    burnStartTime);
    return outputToC(out);
}
