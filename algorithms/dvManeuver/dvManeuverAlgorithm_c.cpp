#include "dvManeuverAlgorithm_c.h"
#include "dvManeuverAlgorithm.h"
#include "dvManeuverTypes.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/opaqueHandle.h"

#include <Eigen/Core>

static_assert(DV_MANEUVER_BURN_STATE_PENDING_C == static_cast<int>(DvManeuverBurnState::Pending),
              "DvManeuverBurnState_c must match DvManeuverBurnState");
static_assert(DV_MANEUVER_BURN_STATE_EXECUTING_C == static_cast<int>(DvManeuverBurnState::Executing),
              "DvManeuverBurnState_c must match DvManeuverBurnState");
static_assert(DV_MANEUVER_BURN_STATE_COMPLETE_C == static_cast<int>(DvManeuverBurnState::Complete),
              "DvManeuverBurnState_c must match DvManeuverBurnState");

namespace {
DvManeuverOutput_c outputToC(const DvManeuverOutput& out) {
    DvManeuverOutput_c result{};
    result.state = static_cast<DvManeuverBurnState_c>(out.state);
    eigenVectorToCArray(out.cmdForce_B, result.cmdForce_B.data);
    return result;
}

DvManeuverConfig configFromC(uint64_t minTime,
                             uint64_t maxTime,
                             float controlPeriod,
                             const Vector3f_c* cmdForce_B,
                             const Vector3f_c* cmdDv_N,
                             uint64_t burnStartTime) {
    return DvManeuverConfig::create(minTime,
                                    maxTime,
                                    controlPeriod,
                                    cArrayToEigenVector3<float>(cmdForce_B->data),
                                    cArrayToEigenVector3<float>(cmdDv_N->data),
                                    burnStartTime);
}
}  // namespace

bool DvManeuverAlgorithm_validateConfig(uint64_t minTime,
                                        uint64_t maxTime,
                                        float controlPeriod,
                                        const Vector3f_c* cmdForce_B,
                                        const Vector3f_c* cmdDv_N,
                                        uint64_t burnStartTime) {
    try {
        (void)configFromC(minTime, maxTime, controlPeriod, cmdForce_B, cmdDv_N, burnStartTime);
        return true;
    } catch (const fsw::invalid_argument&) {
        return false;
    }
}

DvManeuverAlgorithmHandle* DvManeuverAlgorithm_create(uint64_t minTime,
                                                      uint64_t maxTime,
                                                      float controlPeriod,
                                                      const Vector3f_c* cmdForce_B,
                                                      const Vector3f_c* cmdDv_N,
                                                      uint64_t burnStartTime) {
    return reinterpret_cast<DvManeuverAlgorithmHandle*>(
        new ::DvManeuverAlgorithm(configFromC(minTime, maxTime, controlPeriod, cmdForce_B, cmdDv_N, burnStartTime)));
}

void DvManeuverAlgorithm_destroy(DvManeuverAlgorithmHandle* self) { fsw::deleteHandle<::DvManeuverAlgorithm>(self); }

void DvManeuverAlgorithm_setConfig(DvManeuverAlgorithmHandle* self,
                                   uint64_t minTime,
                                   uint64_t maxTime,
                                   float controlPeriod,
                                   const Vector3f_c* cmdForce_B,
                                   const Vector3f_c* cmdDv_N,
                                   uint64_t burnStartTime) {
    fsw::fromHandle<::DvManeuverAlgorithm>(self)->setConfig(
        configFromC(minTime, maxTime, controlPeriod, cmdForce_B, cmdDv_N, burnStartTime));
}

void DvManeuverAlgorithm_reInitialize(DvManeuverAlgorithmHandle* self) {
    fsw::fromHandle<::DvManeuverAlgorithm>(self)->reInitialize();
}

DvManeuverOutput_c DvManeuverAlgorithm_update(DvManeuverAlgorithmHandle* self,
                                              uint64_t callTime,
                                              const Vector3f_c* dvAccumulated) {
    const DvManeuverOutput out = fsw::fromHandle<::DvManeuverAlgorithm>(self)->update(
        callTime, cArrayToEigenVector3<float>(dvAccumulated->data));
    return outputToC(out);
}
