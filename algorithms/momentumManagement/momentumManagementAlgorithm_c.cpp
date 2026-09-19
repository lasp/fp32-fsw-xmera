#include "momentumManagementAlgorithm_c.h"
#include "momentumManagementAlgorithm.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/opaqueHandle.h"

#include <Eigen/Core>

namespace {

//! Build the validated C++ configuration from the flattened C parameters.
MomentumManagementConfig makeConfig(float hsMin,
                                    float K,
                                    float Ki,
                                    float integralLimit,
                                    float controlPeriod,
                                    const Matrix3f_c* dumpableProjection_B,
                                    const MomentumManagementRwArrayConfiguration_c* rwArrayConfig) {
    MomentumManagementRwArrayConfiguration rwArrayConfigCpp;
    rwArrayConfigCpp.GsMatrix_B = cArrayToEigenMatrix<float, 3, kMaxNumRw>(rwArrayConfig->GsMatrix_B);
    rwArrayConfigCpp.JsList = cArrayToEigenVector(rwArrayConfig->JsList);

    const MomentumManagementControlParameters controlParameters{
        .hsMin = hsMin,
        .K = K,
        .Ki = Ki,
        .integralLimit = integralLimit,
        .controlPeriod = controlPeriod,
        .dumpableProjection_B = c2DArrayToEigenMatrix3(dumpableProjection_B->data)};

    return MomentumManagementConfig::create(controlParameters, rwArrayConfigCpp);
}

}  // namespace

uint32_t MomentumManagementAlgorithm_getMaxNumRw(void) { return kMaxNumRw; }

bool MomentumManagementAlgorithm_validateConfig(float hsMin,
                                                float K,
                                                float Ki,
                                                float integralLimit,
                                                float controlPeriod,
                                                const Matrix3f_c* dumpableProjection_B,
                                                const MomentumManagementRwArrayConfiguration_c* rwArrayConfig) {
    try {
        (void)makeConfig(hsMin, K, Ki, integralLimit, controlPeriod, dumpableProjection_B, rwArrayConfig);
        return true;
    } catch (const fsw::invalid_argument&) {
        return false;
    }
}

MomentumManagementAlgorithmHandle* MomentumManagementAlgorithm_create(
    float hsMin,
    float K,
    float Ki,
    float integralLimit,
    float controlPeriod,
    const Matrix3f_c* dumpableProjection_B,
    const MomentumManagementRwArrayConfiguration_c* rwArrayConfig) {
    return fsw::createHandle<::MomentumManagementAlgorithm, MomentumManagementAlgorithmHandle>(
        makeConfig(hsMin, K, Ki, integralLimit, controlPeriod, dumpableProjection_B, rwArrayConfig));
}

void MomentumManagementAlgorithm_destroy(MomentumManagementAlgorithmHandle* self) {
    fsw::deleteHandle<::MomentumManagementAlgorithm>(self);
}

void MomentumManagementAlgorithm_setConfig(MomentumManagementAlgorithmHandle* self,
                                           float hsMin,
                                           float K,
                                           float Ki,
                                           float integralLimit,
                                           float controlPeriod,
                                           const Matrix3f_c* dumpableProjection_B,
                                           const MomentumManagementRwArrayConfiguration_c* rwArrayConfig) {
    fsw::fromHandle<::MomentumManagementAlgorithm>(self)->setConfig(
        makeConfig(hsMin, K, Ki, integralLimit, controlPeriod, dumpableProjection_B, rwArrayConfig));
}

void MomentumManagementAlgorithm_reInitialize(MomentumManagementAlgorithmHandle* self) {
    fsw::fromHandle<::MomentumManagementAlgorithm>(self)->reInitialize();
}

Vector3f_c MomentumManagementAlgorithm_update(MomentumManagementAlgorithmHandle* self,
                                              const MomentumManagementWheelSpeeds_c* wheelSpeeds) {
    const Eigen::Vector<float, kMaxNumRw> wheelSpeedsCpp = cArrayToEigenVector(wheelSpeeds->wheelSpeeds);

    const Eigen::Vector3f Lr_B = fsw::fromHandle<::MomentumManagementAlgorithm>(self)->update(wheelSpeedsCpp);

    Vector3f_c out{};
    eigenVectorToCArray(Lr_B, out.data);

    return out;
}
