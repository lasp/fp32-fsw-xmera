#include "dvGuidanceAlgorithm_c.h"
#include "architecture/utilities/eigenSupport.h"
#include "dvGuidanceAlgorithm.h"

#include <Eigen/Core>

DvGuidanceAlgorithmHandle* DvGuidanceAlgorithm_create(void) {
    // clang-format off
    return reinterpret_cast<DvGuidanceAlgorithmHandle*>(new ::DvGuidanceAlgorithm(DvGuidanceConfig::create()));  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast, cppcoreguidelines-owning-memory)
    // clang-format on
}

void DvGuidanceAlgorithm_destroy(DvGuidanceAlgorithmHandle* self) {
    // clang-format off
    delete reinterpret_cast<::DvGuidanceAlgorithm*>(self);  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast, cppcoreguidelines-owning-memory)
    // clang-format on
}

AttRefMsgF32Payload DvGuidanceAlgorithm_update(const DvGuidanceAlgorithmHandle* self,
                                               Vector3f_c dvInrtlCmd,
                                               Vector3f_c dvRotVecUnit,
                                               float dvRotVecMag,
                                               uint64_t burnStartTime,
                                               uint64_t callTime) {
    const Eigen::Vector3f dvInrtlCmd_e = cArrayToEigenVector3<float>(dvInrtlCmd.data);
    const Eigen::Vector3f dvRotVecUnit_e = cArrayToEigenVector3<float>(dvRotVecUnit.data);

    // clang-format off
    const DvGuidanceOutput out = reinterpret_cast<const ::DvGuidanceAlgorithm*>(self)->update(dvInrtlCmd_e, dvRotVecUnit_e, dvRotVecMag, burnStartTime, callTime);  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    // clang-format on

    AttRefMsgF32Payload payload{};
    eigenVectorToCArray(out.sigma_RN, payload.sigma_RN);
    eigenVectorToCArray(out.omega_RN_N, payload.omega_RN_N);
    eigenVectorToCArray(out.domega_RN_N, payload.domega_RN_N);
    return payload;
}
