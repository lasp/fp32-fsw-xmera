#include "forceTorqueThrForceMappingAlgorithm_c.h"

#include "forceTorqueThrForceMappingAlgorithm.h"
#include "forceTorqueThrForceMappingTypes.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/opaqueHandle.h"

#include <Eigen/Core>
#include <array>
#include <cstdint>

namespace {

ForceTorqueThrForceMappingConfig configFromC(const ForceTorqueThrForceMappingConfig_c& c) {
    ThrusterArrayConfiguration cppThrusters{};
    cppThrusters.numThrusters = c.thrusters.numThrusters;
    for (uint32_t i = 0; i < c.thrusters.numThrusters; ++i) {
        for (uint32_t j = 0; j < 3; ++j) {
            cppThrusters.thrusters.at(i).r_TB_B.at(j) = c.thrusters.thrusters[i].r_TB_B.data[j];
            cppThrusters.thrusters.at(i).tHat_B.at(j) = c.thrusters.thrusters[i].tHat_B.data[j];
        }
    }
    std::array<bool, 6> cppAxes{};
    for (uint32_t i = 0; i < 6; ++i) {
        cppAxes.at(i) = (c.desiredControlAxes[i] != 0);
    }
    return ForceTorqueThrForceMappingConfig::create(
        cppThrusters, cArrayToEigenVector3<float>(c.centerOfMass_B.data), cppAxes);
}

}  // namespace

uint32_t ForceTorqueThrForceMappingAlgorithm_getMaxEffCnt(void) { return MAX_EFF_CNT; }

ForceTorqueThrForceMappingAlgorithmHandle* ForceTorqueThrForceMappingAlgorithm_create(
    const ForceTorqueThrForceMappingConfig_c* config) {
    return fsw::createHandle<::ForceTorqueThrForceMappingAlgorithm, ForceTorqueThrForceMappingAlgorithmHandle>(
        configFromC(*config));
}

void ForceTorqueThrForceMappingAlgorithm_destroy(ForceTorqueThrForceMappingAlgorithmHandle* self) {
    fsw::deleteHandle<::ForceTorqueThrForceMappingAlgorithm>(self);
}

void ForceTorqueThrForceMappingAlgorithm_setConfig(ForceTorqueThrForceMappingAlgorithmHandle* self,
                                                   const ForceTorqueThrForceMappingConfig_c* config) {
    fsw::fromHandle<::ForceTorqueThrForceMappingAlgorithm>(self)->setConfig(configFromC(*config));
}

ThrForceArray_c ForceTorqueThrForceMappingAlgorithm_update(const ForceTorqueThrForceMappingAlgorithmHandle* self,
                                                           const Vector3f_c cmdTorque_B,
                                                           const Vector3f_c cmdForce_B) {
    const Eigen::Vector<float, MAX_EFF_CNT> out =
        fsw::fromHandle<const ::ForceTorqueThrForceMappingAlgorithm>(self)->update(
            cArrayToEigenVector3<float>(cmdTorque_B.data), cArrayToEigenVector3<float>(cmdForce_B.data));

    ThrForceArray_c result{};
    eigenVectorToCArray(out, result.thrForce);
    return result;
}
