#include "convertStPlatformToBodyAlgorithm_c.h"
#include "convertStPlatformToBodyAlgorithm.h"
#include "convertStPlatformToBodyTypes.h"
#include "utilities/fsw/eigenSupport.h"

#include <Eigen/Core>

namespace {
ConvertStPlatformToBodyConfig configFromC(const ConvertStPlatformToBodyConfig_c& c) {
    return ConvertStPlatformToBodyConfig::create(c2DArrayToEigenMatrix3(c.dcm_CB.data));
}
}  // namespace

ConvertStPlatformToBodyAlgorithmHandle* ConvertStPlatformToBodyAlgorithm_create(
    const ConvertStPlatformToBodyConfig_c* config) {
    return reinterpret_cast<ConvertStPlatformToBodyAlgorithmHandle*>(
        new ::ConvertStPlatformToBodyAlgorithm(configFromC(*config)));
}

void ConvertStPlatformToBodyAlgorithm_destroy(ConvertStPlatformToBodyAlgorithmHandle* self) {
    delete reinterpret_cast<::ConvertStPlatformToBodyAlgorithm*>(self);
}

void ConvertStPlatformToBodyAlgorithm_setConfig(ConvertStPlatformToBodyAlgorithmHandle* self,
                                                const ConvertStPlatformToBodyConfig_c* config) {
    reinterpret_cast<::ConvertStPlatformToBodyAlgorithm*>(self)->setConfig(configFromC(*config));
}

StAttitudeOutput_c ConvertStPlatformToBodyAlgorithm_update(ConvertStPlatformToBodyAlgorithmHandle* self,
                                                           const PlatformAttitude_c* platformAttitude,
                                                           const PlatformAngularVelocity_c* platformAngularRate) {
    const Eigen::Vector4f q_CN = cArrayToEigenVector(platformAttitude->q_CN);
    const Eigen::Vector4f dq_CN = cArrayToEigenVector(platformAngularRate->dq_CN);

    const StAttitudeOutput out = reinterpret_cast<::ConvertStPlatformToBodyAlgorithm*>(self)->update(q_CN, dq_CN);

    StAttitudeOutput_c result{};
    eigenVectorToCArray(out.sigma_BN, result.sigma_BN);
    eigenVectorToCArray(out.omega_BN_B, result.omega_BN_B);
    return result;
}
