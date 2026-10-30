#include "convertStPlatformToBodyAlgorithm_c.h"
#include "convertStPlatformToBodyAlgorithm.h"
#include "convertStPlatformToBodyTypes.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/opaqueHandle.h"

#include <Eigen/Core>

namespace {
ConvertStPlatformToBodyConfig configFromC(const Vector3f_c& sigma_PB) {
    return ConvertStPlatformToBodyConfig::create(cArrayToEigenVector3<float>(sigma_PB.data));
}
}  // namespace

bool ConvertStPlatformToBodyAlgorithm_validateConfig(const Vector3f_c sigma_PB) {
    // Attempt to build the config through the real create path; success means valid,
    // a throw means invalid. Reusing configFromC keeps validation from drifting.
    try {
        (void)configFromC(sigma_PB);
        return true;
    } catch (const fsw::invalid_argument&) {
        return false;
    }
}

ConvertStPlatformToBodyAlgorithmHandle* ConvertStPlatformToBodyAlgorithm_create(const Vector3f_c sigma_PB) {
    return fsw::createHandle<::ConvertStPlatformToBodyAlgorithm, ConvertStPlatformToBodyAlgorithmHandle>(
        configFromC(sigma_PB));
}

void ConvertStPlatformToBodyAlgorithm_destroy(ConvertStPlatformToBodyAlgorithmHandle* self) {
    fsw::deleteHandle<::ConvertStPlatformToBodyAlgorithm>(self);
}

void ConvertStPlatformToBodyAlgorithm_setConfig(ConvertStPlatformToBodyAlgorithmHandle* self,
                                                const Vector3f_c sigma_PB) {
    fsw::fromHandle<::ConvertStPlatformToBodyAlgorithm>(self)->setConfig(configFromC(sigma_PB));
}

StAttitudeOutput_c ConvertStPlatformToBodyAlgorithm_update(ConvertStPlatformToBodyAlgorithmHandle* self,
                                                           const StPlatformMeasurement_c* measurement) {
    const Eigen::Vector4f q_PN = cArrayToEigenVector(measurement->q_PN);
    const Eigen::Vector4f dq_PN = cArrayToEigenVector(measurement->dq_PN);

    const StAttitudeOutput out = fsw::fromHandle<::ConvertStPlatformToBodyAlgorithm>(self)->update(q_PN, dq_PN);

    StAttitudeOutput_c result{};
    // The algorithm computes only sigma/omega; the measurement time tag is a
    // pass-through from the input to the body-frame attitude output.
    result.timeTag = measurement->timeTag;
    eigenVectorToCArray(out.sigma_BN, result.sigma_BN);
    eigenVectorToCArray(out.omega_BN_B, result.omega_BN_B);
    return result;
}
