#include "forceTorqueThrForceMappingAlgorithm_c.h"

#include "architecture/utilities/eigenSupport.h"
#include "forceTorqueThrForceMappingAlgorithm.h"
#include "forceTorqueThrForceMappingTypes.h"

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

ForceTorqueThrForceMappingAlgorithm* ForceTorqueThrForceMappingAlgorithm_create(
    const ForceTorqueThrForceMappingConfig_c* config) {
    // clang-format off
    return reinterpret_cast<ForceTorqueThrForceMappingAlgorithm*>(new ::ForceTorqueThrForceMappingAlgorithm(configFromC(*config)));  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast, cppcoreguidelines-owning-memory)
    // clang-format on
}

void ForceTorqueThrForceMappingAlgorithm_destroy(ForceTorqueThrForceMappingAlgorithm* self) {
    // clang-format off
    delete reinterpret_cast<::ForceTorqueThrForceMappingAlgorithm*>(self);  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast, cppcoreguidelines-owning-memory)
    // clang-format on
}

void ForceTorqueThrForceMappingAlgorithm_setConfig(ForceTorqueThrForceMappingAlgorithm* self,
                                                   const ForceTorqueThrForceMappingConfig_c* config) {
    // clang-format off
    reinterpret_cast<::ForceTorqueThrForceMappingAlgorithm*>(self)->setConfig(configFromC(*config));  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    // clang-format on
}

ThrForceArray_c ForceTorqueThrForceMappingAlgorithm_update(const ForceTorqueThrForceMappingAlgorithm* self,
                                                           const Vector3f_c cmdTorque_B,
                                                           const Vector3f_c cmdForce_B) {
    // clang-format off
    const Eigen::Vector<float, MAX_EFF_CNT> out = reinterpret_cast<const ::ForceTorqueThrForceMappingAlgorithm*>(self)->update(cArrayToEigenVector3<float>(cmdTorque_B.data), cArrayToEigenVector3<float>(cmdForce_B.data));  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    // clang-format on

    ThrForceArray_c result{};
    eigenVectorToCArray(out, result.thrForce);
    return result;
}
