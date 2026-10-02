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
    eigenVectorToCArray(out.cmdForce_B, result.cmdForce_B.data);
    return result;
}

DvManeuverConfig configFromC(float minTime, float maxTime, float controlPeriod, const Vector3f_c* cmdForce_B) {
    return DvManeuverConfig::create(minTime, maxTime, controlPeriod, cArrayToEigenVector3<float>(cmdForce_B->data));
}
}  // namespace

bool DvManeuverAlgorithm_validateConfig(float minTime,
                                        float maxTime,
                                        float controlPeriod,
                                        const Vector3f_c* cmdForce_B) {
    try {
        (void)configFromC(minTime, maxTime, controlPeriod, cmdForce_B);
        return true;
    } catch (const fsw::invalid_argument&) {
        return false;
    }
}

DvManeuverAlgorithmHandle* DvManeuverAlgorithm_create(float minTime,
                                                      float maxTime,
                                                      float controlPeriod,
                                                      const Vector3f_c* cmdForce_B) {
    return reinterpret_cast<DvManeuverAlgorithmHandle*>(
        new ::DvManeuverAlgorithm(configFromC(minTime, maxTime, controlPeriod, cmdForce_B)));
}

void DvManeuverAlgorithm_destroy(DvManeuverAlgorithmHandle* self) { fsw::deleteHandle<::DvManeuverAlgorithm>(self); }

void DvManeuverAlgorithm_setConfig(DvManeuverAlgorithmHandle* self,
                                   float minTime,
                                   float maxTime,
                                   float controlPeriod,
                                   const Vector3f_c* cmdForce_B) {
    fsw::fromHandle<::DvManeuverAlgorithm>(self)->setConfig(configFromC(minTime, maxTime, controlPeriod, cmdForce_B));
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
