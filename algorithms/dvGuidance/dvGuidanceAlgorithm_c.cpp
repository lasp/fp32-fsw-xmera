#include "dvGuidanceAlgorithm_c.h"
#include "dvGuidanceAlgorithm.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/opaqueHandle.h"

#include <Eigen/Core>

namespace {
DvGuidanceConfig configFromC(const Vector3f_c& cmdDv_N, const Vector3f_c& rHatPref_N) {
    return DvGuidanceConfig::create(cArrayToEigenVector3<float>(cmdDv_N.data),
                                    cArrayToEigenVector3<float>(rHatPref_N.data));
}
}  // namespace

bool DvGuidanceAlgorithm_validateConfig(const Vector3f_c cmdDv_N, const Vector3f_c rHatPref_N) {
    // Attempt to build the config through the real create path; success means valid,
    // a throw means invalid. Reusing configFromC keeps validation from drifting.
    try {
        (void)configFromC(cmdDv_N, rHatPref_N);
        return true;
    } catch (const fsw::invalid_argument&) {
        return false;
    }
}

DvGuidanceAlgorithmHandle* DvGuidanceAlgorithm_create(const Vector3f_c cmdDv_N, const Vector3f_c rHatPref_N) {
    return fsw::createHandle<::DvGuidanceAlgorithm, DvGuidanceAlgorithmHandle>(configFromC(cmdDv_N, rHatPref_N));
}

void DvGuidanceAlgorithm_destroy(DvGuidanceAlgorithmHandle* self) { fsw::deleteHandle<::DvGuidanceAlgorithm>(self); }

void DvGuidanceAlgorithm_setConfig(DvGuidanceAlgorithmHandle* self,
                                   const Vector3f_c cmdDv_N,
                                   const Vector3f_c rHatPref_N) {
    fsw::fromHandle<::DvGuidanceAlgorithm>(self)->setConfig(configFromC(cmdDv_N, rHatPref_N));
}

AttRefMsgF32Payload DvGuidanceAlgorithm_update(const DvGuidanceAlgorithmHandle* self) {
    const Eigen::Vector3f sigma_RN = fsw::fromHandle<const ::DvGuidanceAlgorithm>(self)->update();

    // The burn frame is inertially fixed: the zero-initialized reference rate and acceleration are left at zero.
    AttRefMsgF32Payload payload{};
    eigenVectorToCArray(sigma_RN, payload.sigma_RN);
    return payload;
}
