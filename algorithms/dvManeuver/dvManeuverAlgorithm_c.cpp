#include "dvManeuverAlgorithm_c.h"
#include "dvManeuverAlgorithm.h"
#include "dvManeuverTypes.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/opaqueHandle.h"

#include <Eigen/Core>

namespace {
DvManeuverOutput_c outputToC(const DvManeuverOutput& out) {
    DvManeuverOutput_c result{};
    result.burnExecuting = out.burnExecuting;
    result.burnComplete = out.burnComplete;
    result.commandThrustersOff = out.commandThrustersOff;
    return result;
}
}  // namespace

bool DvManeuverAlgorithm_validateConfig(float minTime, float maxTime, float controlPeriod) {
    try {
        (void)DvManeuverConfig::create(minTime, maxTime, controlPeriod);
        return true;
    } catch (const fsw::invalid_argument&) {
        return false;
    }
}

DvManeuverAlgorithmHandle* DvManeuverAlgorithm_create(float minTime, float maxTime, float controlPeriod) {
    return reinterpret_cast<DvManeuverAlgorithmHandle*>(
        new ::DvManeuverAlgorithm(DvManeuverConfig::create(minTime, maxTime, controlPeriod)));
}

void DvManeuverAlgorithm_destroy(DvManeuverAlgorithmHandle* self) { fsw::deleteHandle<::DvManeuverAlgorithm>(self); }

void DvManeuverAlgorithm_setConfig(DvManeuverAlgorithmHandle* self, float minTime, float maxTime, float controlPeriod) {
    fsw::fromHandle<::DvManeuverAlgorithm>(self)->setConfig(DvManeuverConfig::create(minTime, maxTime, controlPeriod));
}

void DvManeuverAlgorithm_reInitialize(DvManeuverAlgorithmHandle* self) {
    fsw::fromHandle<::DvManeuverAlgorithm>(self)->reInitialize();
}

DvManeuverOutput_c DvManeuverAlgorithm_update(DvManeuverAlgorithmHandle* self,
                                              uint64_t callTime,
                                              const Vector3f_c* vehAccumDV,
                                              const Vector3f_c* dvInrtlCmd,
                                              uint64_t burnStartTime) {
    const DvManeuverOutput out =
        fsw::fromHandle<::DvManeuverAlgorithm>(self)->update(callTime,
                                                             cArrayToEigenVector3<float>(vehAccumDV->data),
                                                             cArrayToEigenVector3<float>(dvInrtlCmd->data),
                                                             burnStartTime);
    return outputToC(out);
}
