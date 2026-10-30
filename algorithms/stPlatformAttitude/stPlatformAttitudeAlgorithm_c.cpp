#include "stPlatformAttitudeAlgorithm_c.h"
#include "stPlatformAttitudeAlgorithm.h"
#include "stPlatformAttitudeTypes.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/opaqueHandle.h"

#include <Eigen/Core>

namespace {
StPlatformAttitudeConfig configFromC(const Vector3f_c& sigma_PB) {
    return StPlatformAttitudeConfig::create(cArrayToEigenVector3<float>(sigma_PB.data));
}
}  // namespace

bool StPlatformAttitudeAlgorithm_validateConfig(const Vector3f_c sigma_PB) {
    // Attempt to build the config through the real create path; success means valid,
    // a throw means invalid. Reusing configFromC keeps validation from drifting.
    try {
        (void)configFromC(sigma_PB);
        return true;
    } catch (const fsw::invalid_argument&) {
        return false;
    }
}

StPlatformAttitudeAlgorithmHandle* StPlatformAttitudeAlgorithm_create(const Vector3f_c sigma_PB) {
    return fsw::createHandle<::StPlatformAttitudeAlgorithm, StPlatformAttitudeAlgorithmHandle>(configFromC(sigma_PB));
}

void StPlatformAttitudeAlgorithm_destroy(StPlatformAttitudeAlgorithmHandle* self) {
    fsw::deleteHandle<::StPlatformAttitudeAlgorithm>(self);
}

void StPlatformAttitudeAlgorithm_setConfig(StPlatformAttitudeAlgorithmHandle* self, const Vector3f_c sigma_PB) {
    fsw::fromHandle<::StPlatformAttitudeAlgorithm>(self)->setConfig(configFromC(sigma_PB));
}

StAttitudeOutput_c StPlatformAttitudeAlgorithm_update(StPlatformAttitudeAlgorithmHandle* self,
                                                      const StPlatformMeasurement_c* measurement) {
    const Eigen::Vector4f q_PN = cArrayToEigenVector(measurement->q_PN);
    const Eigen::Vector4f rateQuaternion_PN = cArrayToEigenVector(measurement->rateQuaternion_PN);

    const StAttitudeOutput out = fsw::fromHandle<::StPlatformAttitudeAlgorithm>(self)->update(q_PN, rateQuaternion_PN);

    StAttitudeOutput_c result{};
    // The algorithm computes only sigma/omega; the measurement time tag is a
    // pass-through from the input to the body-frame attitude output.
    result.timeTag = measurement->timeTag;
    eigenVectorToCArray(out.sigma_BN, result.sigma_BN);
    eigenVectorToCArray(out.omega_BN_B, result.omega_BN_B);
    return result;
}
