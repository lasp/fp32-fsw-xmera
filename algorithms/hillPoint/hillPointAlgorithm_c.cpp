#include "hillPointAlgorithm_c.h"
#include "hillPointAlgorithm.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/opaqueHandle.h"

#include <Eigen/Core>

HillPointAlgorithmHandle* HillPointAlgorithm_create(void) {
    return fsw::createHandle<::HillPointAlgorithm, HillPointAlgorithmHandle>();
}

void HillPointAlgorithm_destroy(HillPointAlgorithmHandle* self) { fsw::deleteHandle<::HillPointAlgorithm>(self); }

AttRefMsgF32Payload HillPointAlgorithm_update(const HillPointAlgorithmHandle* self,
                                              Vector3d_c r_BN_N,
                                              Vector3d_c v_BN_N,
                                              Vector3d_c r_PN_N,
                                              Vector3d_c v_PN_N) {
    const Eigen::Vector3d r_BN_N_e = cArrayToEigenVector3<double>(r_BN_N.data);
    const Eigen::Vector3d v_BN_N_e = cArrayToEigenVector3<double>(v_BN_N.data);
    const Eigen::Vector3d r_PN_N_e = cArrayToEigenVector3<double>(r_PN_N.data);
    const Eigen::Vector3d v_PN_N_e = cArrayToEigenVector3<double>(v_PN_N.data);

    const HillPointOutput out =
        fsw::fromHandle<const ::HillPointAlgorithm>(self)->update(r_BN_N_e, v_BN_N_e, r_PN_N_e, v_PN_N_e);

    AttRefMsgF32Payload payload{};
    eigenVectorToCArray(out.sigma_RN, payload.sigma_RN);
    eigenVectorToCArray(out.omega_RN_N, payload.omega_RN_N);
    eigenVectorToCArray(out.domega_RN_N, payload.domega_RN_N);
    return payload;
}
