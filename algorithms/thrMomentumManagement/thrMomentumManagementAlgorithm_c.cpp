#include "thrMomentumManagementAlgorithm_c.h"
#include "thrMomentumManagementAlgorithm.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/opaqueHandle.h"

#include <Eigen/Core>

namespace {

//! Build the validated C++ configuration from the flattened C parameters.
ThrMomentumManagementConfig makeConfig(float hsMin, const ThrMomentumManagementRwArrayConfiguration_c* rwArrayConfig) {
    ThrMomentumManagementRwArrayConfiguration rwArrayConfigCpp;
    rwArrayConfigCpp.numRW = rwArrayConfig->numRW;
    rwArrayConfigCpp.GsMatrix_B = cArrayToEigenMatrix<float, 3, kMaxNumRw>(rwArrayConfig->GsMatrix_B);
    rwArrayConfigCpp.JsList = cArrayToEigenVector(rwArrayConfig->JsList);

    return ThrMomentumManagementConfig::create(hsMin, rwArrayConfigCpp);
}

}  // namespace

uint32_t ThrMomentumManagementAlgorithm_getMaxNumRw(void) { return THR_MOMENTUM_MANAGEMENT_MAX_NUM_RW; }

bool ThrMomentumManagementAlgorithm_validateConfig(float hsMin,
                                                   const ThrMomentumManagementRwArrayConfiguration_c* rwArrayConfig) {
    try {
        (void)makeConfig(hsMin, rwArrayConfig);
        return true;
    } catch (const fsw::invalid_argument&) {
        return false;
    }
}

ThrMomentumManagementAlgorithmHandle* ThrMomentumManagementAlgorithm_create(
    float hsMin,
    const ThrMomentumManagementRwArrayConfiguration_c* rwArrayConfig) {
    return fsw::createHandle<::ThrMomentumManagementAlgorithm, ThrMomentumManagementAlgorithmHandle>(
        makeConfig(hsMin, rwArrayConfig));
}

void ThrMomentumManagementAlgorithm_destroy(ThrMomentumManagementAlgorithmHandle* self) {
    fsw::deleteHandle<::ThrMomentumManagementAlgorithm>(self);
}

void ThrMomentumManagementAlgorithm_setConfig(ThrMomentumManagementAlgorithmHandle* self,
                                              float hsMin,
                                              const ThrMomentumManagementRwArrayConfiguration_c* rwArrayConfig) {
    fsw::fromHandle<::ThrMomentumManagementAlgorithm>(self)->setConfig(makeConfig(hsMin, rwArrayConfig));
}

Vector3f_c ThrMomentumManagementAlgorithm_update(const ThrMomentumManagementAlgorithmHandle* self,
                                                 const ThrMomentumManagementWheelSpeeds_c* wheelSpeeds) {
    const Eigen::Vector<float, kMaxNumRw> wheelSpeedsCpp = cArrayToEigenVector(wheelSpeeds->wheelSpeeds);

    const Eigen::Vector3f Delta_H_B =
        fsw::fromHandle<const ::ThrMomentumManagementAlgorithm>(self)->update(wheelSpeedsCpp);

    Vector3f_c out{};
    eigenVectorToCArray(Delta_H_B, out.data);

    return out;
}
