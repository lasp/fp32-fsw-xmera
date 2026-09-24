#include "forceTorqueThrForceMappingAlgorithm_c.h"

#include "forceTorqueThrForceMappingAlgorithm.h"
#include "forceTorqueThrForceMappingTypes.h"
#include "utilities/fsw/deviceAvailability.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/opaqueHandle.h"

#include <Eigen/Core>
#include <array>
#include <cstdint>

namespace {

// Reassemble the flattened C arguments into the C++ configuration structs. The flat argument
// list is the shape of the extern "C" boundary only; everything behind this helper is struct
// based, and ForceTorqueThrForceMappingConfig::create remains the single validation authority.
ForceTorqueThrForceMappingConfig configFromC(uint32_t numThrusters,
                                             const ThrusterGeometryArray_c& rThruster_B,
                                             const ThrusterGeometryArray_c& tHatThruster_B,
                                             const Vector3f_c& centerOfMass_B,
                                             const ForceTorqueControlAxes_c& desiredControlAxes_B,
                                             const ThrusterAvailabilityArray_c& thrusterAvailability) {
    ThrusterArrayConfiguration thrusters{};
    thrusters.numThrusters = numThrusters;
    // Every slot is filled regardless of the count: create() reads only the first numThrusters of them,
    // and leaving the rest at zero would otherwise depend on the caller's padding.
    for (uint32_t i = 0; i < kMaxThrusterCount; ++i) {
        for (uint32_t j = 0; j < 3U; ++j) {
            thrusters.thrusters.at(i).r_TB_B.at(j) = rThruster_B.data[(i * 3U) + j];
            thrusters.thrusters.at(i).tHat_B.at(j) = tHatThruster_B.data[(i * 3U) + j];
        }
        // An unscoped C enum can carry a value outside its enumerators. Converting through
        // toDeviceAvailability keeps any such value out.
        thrusters.thrusterAvailability.at(i) = fsw::toDeviceAvailability(thrusterAvailability.availability[i]);
    }

    const std::array<bool, 6> axes{desiredControlAxes_B.torqueX,
                                   desiredControlAxes_B.torqueY,
                                   desiredControlAxes_B.torqueZ,
                                   desiredControlAxes_B.forceX,
                                   desiredControlAxes_B.forceY,
                                   desiredControlAxes_B.forceZ};

    return ForceTorqueThrForceMappingConfig::create(thrusters, cArrayToEigenVector3<float>(centerOfMass_B.data), axes);
}

}  // namespace

uint32_t ForceTorqueThrForceMappingAlgorithm_getMaxThrusterCount(void) { return kMaxThrusterCount; }

bool ForceTorqueThrForceMappingAlgorithm_validateConfig(uint32_t numThrusters,
                                                        const ThrusterGeometryArray_c* rThruster_B,
                                                        const ThrusterGeometryArray_c* tHatThruster_B,
                                                        const Vector3f_c* centerOfMass_B,
                                                        const ForceTorqueControlAxes_c* desiredControlAxes_B,
                                                        const ThrusterAvailabilityArray_c* thrusterAvailability) {
    // Attempt to build the config through the real create path (configFromC ->
    // ForceTorqueThrForceMappingConfig::create): success means valid, a throw means invalid.
    // Reusing create means this validation can never drift from the rules it enforces.
    try {
        (void)configFromC(
            numThrusters, *rThruster_B, *tHatThruster_B, *centerOfMass_B, *desiredControlAxes_B, *thrusterAvailability);
        return true;
    } catch (const fsw::invalid_argument&) {
        return false;
    }
}

ForceTorqueThrForceMappingAlgorithmHandle* ForceTorqueThrForceMappingAlgorithm_create(
    uint32_t numThrusters,
    const ThrusterGeometryArray_c* rThruster_B,
    const ThrusterGeometryArray_c* tHatThruster_B,
    const Vector3f_c* centerOfMass_B,
    const ForceTorqueControlAxes_c* desiredControlAxes_B,
    const ThrusterAvailabilityArray_c* thrusterAvailability) {
    return fsw::createHandle<::ForceTorqueThrForceMappingAlgorithm,
                             ForceTorqueThrForceMappingAlgorithmHandle>(configFromC(
        numThrusters, *rThruster_B, *tHatThruster_B, *centerOfMass_B, *desiredControlAxes_B, *thrusterAvailability));
}

void ForceTorqueThrForceMappingAlgorithm_destroy(ForceTorqueThrForceMappingAlgorithmHandle* self) {
    fsw::deleteHandle<::ForceTorqueThrForceMappingAlgorithm>(self);
}

void ForceTorqueThrForceMappingAlgorithm_setConfig(ForceTorqueThrForceMappingAlgorithmHandle* self,
                                                   uint32_t numThrusters,
                                                   const ThrusterGeometryArray_c* rThruster_B,
                                                   const ThrusterGeometryArray_c* tHatThruster_B,
                                                   const Vector3f_c* centerOfMass_B,
                                                   const ForceTorqueControlAxes_c* desiredControlAxes_B,
                                                   const ThrusterAvailabilityArray_c* thrusterAvailability) {
    fsw::fromHandle<::ForceTorqueThrForceMappingAlgorithm>(self)->setConfig(configFromC(
        numThrusters, *rThruster_B, *tHatThruster_B, *centerOfMass_B, *desiredControlAxes_B, *thrusterAvailability));
}

ThrForceArray_c ForceTorqueThrForceMappingAlgorithm_update(const ForceTorqueThrForceMappingAlgorithmHandle* self,
                                                           const Vector3f_c* cmdTorque_B,
                                                           const Vector3f_c* cmdForce_B) {
    const Eigen::Vector<float, kMaxThrusterCount> out =
        fsw::fromHandle<const ::ForceTorqueThrForceMappingAlgorithm>(self)->update(
            cArrayToEigenVector3<float>(cmdTorque_B->data), cArrayToEigenVector3<float>(cmdForce_B->data));

    ThrForceArray_c result{};
    eigenVectorToCArray(out, result.thrForce);
    return result;
}
