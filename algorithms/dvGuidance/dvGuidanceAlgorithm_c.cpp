#include "dvGuidanceAlgorithm_c.h"
#include "dvGuidanceAlgorithm.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/opaqueHandle.h"

#include <Eigen/Core>

DvGuidanceAlgorithmHandle* DvGuidanceAlgorithm_create(void) {
    return fsw::createHandle<::DvGuidanceAlgorithm, DvGuidanceAlgorithmHandle>();
}

void DvGuidanceAlgorithm_destroy(DvGuidanceAlgorithmHandle* self) { fsw::deleteHandle<::DvGuidanceAlgorithm>(self); }

AttRefMsgF32Payload DvGuidanceAlgorithm_update(const DvGuidanceAlgorithmHandle* self,
                                               Vector3f_c dvInrtlCmd,
                                               Vector3f_c dvRotVecUnit,
                                               float dvRotVecMag,
                                               uint64_t burnStartTime,
                                               uint64_t callTime) {
    const Eigen::Vector3f dvInrtlCmd_e = cArrayToEigenVector3<float>(dvInrtlCmd.data);
    const Eigen::Vector3f dvRotVecUnit_e = cArrayToEigenVector3<float>(dvRotVecUnit.data);

    const DvGuidanceOutput out = fsw::fromHandle<const ::DvGuidanceAlgorithm>(self)->update(
        dvInrtlCmd_e, dvRotVecUnit_e, dvRotVecMag, burnStartTime, callTime);

    AttRefMsgF32Payload payload{};
    eigenVectorToCArray(out.sigma_RN, payload.sigma_RN);
    eigenVectorToCArray(out.omega_RN_N, payload.omega_RN_N);
    eigenVectorToCArray(out.domega_RN_N, payload.domega_RN_N);
    return payload;
}
