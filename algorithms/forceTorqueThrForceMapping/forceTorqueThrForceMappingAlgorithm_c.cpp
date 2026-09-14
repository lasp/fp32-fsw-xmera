#include "forceTorqueThrForceMappingAlgorithm_c.h"

#include "forceTorqueThrForceMappingAlgorithm.h"
#include "forceTorqueThrForceMappingTypes.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/opaqueHandle.h"

#include <Eigen/Core>
#include <array>
#include <cstdint>

namespace {

// Reassemble the flattened C arguments into the C++ configuration structs. The flat argument
// list is the shape of the extern "C" boundary only; everything behind this helper is struct
// based, and ForceTorqueThrForceMappingConfig::create remains the single validation authority.
ForceTorqueThrForceMappingConfig configFromC(const float rThruster_B[MAX_EFF_CNT * 3],
                                             const float tHatThruster_B[MAX_EFF_CNT * 3],
                                             const float centerOfMass_B[3],
                                             const ForceTorqueControlAxes_c& desiredControlAxes_B) {
    ThrusterArrayConfiguration thrusters{};
    for (uint32_t i = 0; i < kMaxThrusterCount; ++i) {
        for (uint32_t j = 0; j < 3U; ++j) {
            thrusters.thrusters.at(i).r_TB_B.at(j) = rThruster_B[(i * 3U) + j];
            thrusters.thrusters.at(i).tHat_B.at(j) = tHatThruster_B[(i * 3U) + j];
        }
    }

    const std::array<bool, 6> axes{desiredControlAxes_B.torqueX,
                                   desiredControlAxes_B.torqueY,
                                   desiredControlAxes_B.torqueZ,
                                   desiredControlAxes_B.forceX,
                                   desiredControlAxes_B.forceY,
                                   desiredControlAxes_B.forceZ};

    return ForceTorqueThrForceMappingConfig::create(thrusters, cArrayToEigenVector3<float>(centerOfMass_B), axes);
}

}  // namespace

uint32_t ForceTorqueThrForceMappingAlgorithm_getMaxThrusterCount(void) { return kMaxThrusterCount; }

ForceTorqueThrForceMappingAlgorithmHandle* ForceTorqueThrForceMappingAlgorithm_create(
    float rThruster_B[MAX_EFF_CNT * 3],
    float tHatThruster_B[MAX_EFF_CNT * 3],
    float centerOfMass_B[3],
    const ForceTorqueControlAxes_c* desiredControlAxes_B) {
    return fsw::createHandle<::ForceTorqueThrForceMappingAlgorithm, ForceTorqueThrForceMappingAlgorithmHandle>(
        configFromC(rThruster_B, tHatThruster_B, centerOfMass_B, *desiredControlAxes_B));
}

void ForceTorqueThrForceMappingAlgorithm_destroy(ForceTorqueThrForceMappingAlgorithmHandle* self) {
    fsw::deleteHandle<::ForceTorqueThrForceMappingAlgorithm>(self);
}

void ForceTorqueThrForceMappingAlgorithm_setConfig(ForceTorqueThrForceMappingAlgorithmHandle* self,
                                                   float rThruster_B[MAX_EFF_CNT * 3],
                                                   float tHatThruster_B[MAX_EFF_CNT * 3],
                                                   float centerOfMass_B[3],
                                                   const ForceTorqueControlAxes_c* desiredControlAxes_B) {
    fsw::fromHandle<::ForceTorqueThrForceMappingAlgorithm>(self)->setConfig(
        configFromC(rThruster_B, tHatThruster_B, centerOfMass_B, *desiredControlAxes_B));
}

ThrForceArray_c ForceTorqueThrForceMappingAlgorithm_update(const ForceTorqueThrForceMappingAlgorithmHandle* self,
                                                           float cmdTorque_B[3],
                                                           float cmdForce_B[3]) {
    const Eigen::Vector<float, kMaxThrusterCount> out =
        fsw::fromHandle<const ::ForceTorqueThrForceMappingAlgorithm>(self)->update(
            cArrayToEigenVector3<float>(cmdTorque_B), cArrayToEigenVector3<float>(cmdForce_B));

    ThrForceArray_c result{};
    eigenVectorToCArray(out, result.thrForce);
    return result;
}
