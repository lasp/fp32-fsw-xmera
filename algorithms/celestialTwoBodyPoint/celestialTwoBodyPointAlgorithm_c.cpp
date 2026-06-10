#include "celestialTwoBodyPointAlgorithm_c.h"
#include "architecture/utilities/eigenSupport.h"
#include "celestialTwoBodyPointAlgorithm.h"

#include <Eigen/Core>

namespace {
CelestialTwoBodyPointConfig configFromC(const CelestialTwoBodyPointConfig_c& c) {
    return CelestialTwoBodyPointConfig::create(c.celestialBodyAlignmentThreshold);
}
}  // namespace

CelestialTwoBodyPointAlgorithmHandle* CelestialTwoBodyPointAlgorithm_create(
    const CelestialTwoBodyPointConfig_c* config) {
    // clang-format off
    return reinterpret_cast<CelestialTwoBodyPointAlgorithmHandle*>(new ::CelestialTwoBodyPointAlgorithm(configFromC(*config)));  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast, cppcoreguidelines-owning-memory)
    // clang-format on
}

void CelestialTwoBodyPointAlgorithm_destroy(CelestialTwoBodyPointAlgorithmHandle* self) {
    // clang-format off
    delete reinterpret_cast<::CelestialTwoBodyPointAlgorithm*>(self);  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast, cppcoreguidelines-owning-memory)
    // clang-format on
}

void CelestialTwoBodyPointAlgorithm_setConfig(CelestialTwoBodyPointAlgorithmHandle* self,
                                              const CelestialTwoBodyPointConfig_c* config) {
    // clang-format off
    reinterpret_cast<::CelestialTwoBodyPointAlgorithm*>(self)->setConfig(configFromC(*config));  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    // clang-format on
}

AttRefMsgF32Payload CelestialTwoBodyPointAlgorithm_update(const CelestialTwoBodyPointAlgorithmHandle* self,
                                                          Vector3d_c r_PN_N,
                                                          Vector3d_c v_PN_N,
                                                          Vector3d_c r_SN_N,
                                                          Vector3d_c v_SN_N,
                                                          Vector3d_c r_BN_N,
                                                          Vector3d_c v_BN_N) {
    const Eigen::Vector3d r_PN_N_e = cArrayToEigenVector3<double>(r_PN_N.data);
    const Eigen::Vector3d v_PN_N_e = cArrayToEigenVector3<double>(v_PN_N.data);
    const Eigen::Vector3d r_SN_N_e = cArrayToEigenVector3<double>(r_SN_N.data);
    const Eigen::Vector3d v_SN_N_e = cArrayToEigenVector3<double>(v_SN_N.data);
    const Eigen::Vector3d r_BN_N_e = cArrayToEigenVector3<double>(r_BN_N.data);
    const Eigen::Vector3d v_BN_N_e = cArrayToEigenVector3<double>(v_BN_N.data);

    // clang-format off
    const CelestialTwoBodyPointOutput out = reinterpret_cast<const ::CelestialTwoBodyPointAlgorithm*>(self)->update(r_PN_N_e, v_PN_N_e, r_SN_N_e, v_SN_N_e, r_BN_N_e, v_BN_N_e);  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    // clang-format on

    AttRefMsgF32Payload payload{};
    eigenVectorToCArray(out.sigma_RN, payload.sigma_RN);
    eigenVectorToCArray(out.omega_RN_N, payload.omega_RN_N);
    eigenVectorToCArray(out.domega_RN_N, payload.domega_RN_N);
    return payload;
}
