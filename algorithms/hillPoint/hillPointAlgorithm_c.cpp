#include "hillPointAlgorithm_c.h"
#include "hillPointAlgorithm.h"
#include "utilities/fsw/eigenSupport.h"

#include <Eigen/Core>

HillPointAlgorithmHandle* HillPointAlgorithm_create(void) {
    // clang-format off
    return reinterpret_cast<HillPointAlgorithmHandle*>(new ::HillPointAlgorithm(HillPointConfig::create()));  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast, cppcoreguidelines-owning-memory)
    // clang-format on
}

void HillPointAlgorithm_destroy(HillPointAlgorithmHandle* self) {
    // clang-format off
    delete reinterpret_cast<::HillPointAlgorithm*>(self);  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast, cppcoreguidelines-owning-memory)
    // clang-format on
}

AttRefMsgF32Payload HillPointAlgorithm_update(const HillPointAlgorithmHandle* self,
                                              Vector3d_c r_BN_N,
                                              Vector3d_c v_BN_N,
                                              Vector3d_c r_planet_N,
                                              Vector3d_c v_planet_N) {
    const Eigen::Vector3d r_BN_N_e = cArrayToEigenVector3<double>(r_BN_N.data);
    const Eigen::Vector3d v_BN_N_e = cArrayToEigenVector3<double>(v_BN_N.data);
    const Eigen::Vector3d r_planet_N_e = cArrayToEigenVector3<double>(r_planet_N.data);
    const Eigen::Vector3d v_planet_N_e = cArrayToEigenVector3<double>(v_planet_N.data);

    // clang-format off
    const HillPointOutput out = reinterpret_cast<const ::HillPointAlgorithm*>(self)->update(r_BN_N_e, v_BN_N_e, r_planet_N_e, v_planet_N_e);  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    // clang-format on

    AttRefMsgF32Payload payload{};
    eigenVectorToCArray(out.sigma_RN, payload.sigma_RN);
    eigenVectorToCArray(out.omega_RN_N, payload.omega_RN_N);
    eigenVectorToCArray(out.domega_RN_N, payload.domega_RN_N);
    return payload;
}
