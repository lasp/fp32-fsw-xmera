#include "convertStPlatformToBodyAlgorithm_c.h"
#include "convertStPlatformToBodyAlgorithm.h"
#include "convertStPlatformToBodyTypes.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/opaqueHandle.h"

#include <Eigen/Core>

namespace {
ConvertStPlatformToBodyConfig configFromC(const Matrix3f_c& dcm_CB) {
    return ConvertStPlatformToBodyConfig::create(c2DArrayToEigenMatrix3(dcm_CB.data));
}
}  // namespace

ConvertStPlatformToBodyAlgorithmHandle* ConvertStPlatformToBodyAlgorithm_create(const Matrix3f_c dcm_CB) {
    return fsw::createHandle<::ConvertStPlatformToBodyAlgorithm, ConvertStPlatformToBodyAlgorithmHandle>(
        configFromC(dcm_CB));
}

void ConvertStPlatformToBodyAlgorithm_destroy(ConvertStPlatformToBodyAlgorithmHandle* self) {
    fsw::deleteHandle<::ConvertStPlatformToBodyAlgorithm>(self);
}

void ConvertStPlatformToBodyAlgorithm_setConfig(ConvertStPlatformToBodyAlgorithmHandle* self, const Matrix3f_c dcm_CB) {
    fsw::fromHandle<::ConvertStPlatformToBodyAlgorithm>(self)->setConfig(configFromC(dcm_CB));
}

StAttitudeOutput_c ConvertStPlatformToBodyAlgorithm_update(ConvertStPlatformToBodyAlgorithmHandle* self,
                                                           const PlatformAttitude_c* platformAttitude,
                                                           const PlatformAngularVelocity_c* platformAngularRate) {
    const Eigen::Vector4f q_CN = cArrayToEigenVector(platformAttitude->q_CN);
    const Eigen::Vector4f dq_CN = cArrayToEigenVector(platformAngularRate->dq_CN);

    const StAttitudeOutput out = fsw::fromHandle<::ConvertStPlatformToBodyAlgorithm>(self)->update(q_CN, dq_CN);

    StAttitudeOutput_c result{};
    // The algorithm computes only sigma/omega; the measurement time tag is a
    // pass-through from the attitude input to the body-frame attitude output.
    result.timeTag = platformAttitude->timeTag;
    eigenVectorToCArray(out.sigma_BN, result.sigma_BN);
    eigenVectorToCArray(out.omega_BN_B, result.omega_BN_B);
    return result;
}
