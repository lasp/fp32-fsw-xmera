#include "celestialTwoBodyPointAlgorithm_c.h"
#include "architecture/utilities/eigenSupport.h"
#include "celestialTwoBodyPointAlgorithm.h"

#include <Eigen/Core>

namespace {
CelestialTwoBodyPointConfig configFromC(const CelestialTwoBodyPointConfig_c& c) {
    return CelestialTwoBodyPointConfig::create(c.singularityThreshold, c.rateThreshold, c.secCelBodyIsLinked != 0);
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
                                                          Vector3d_c r_celBody_N,
                                                          Vector3d_c v_celBody_N,
                                                          Vector3d_c r_secCelBody_N,
                                                          Vector3d_c v_secCelBody_N,
                                                          Vector3d_c r_BN_N,
                                                          Vector3d_c v_BN_N) {
    const Eigen::Vector3d r_celBody_N_e = cArrayToEigenVector3<double>(r_celBody_N.data);
    const Eigen::Vector3d v_celBody_N_e = cArrayToEigenVector3<double>(v_celBody_N.data);
    const Eigen::Vector3d r_secCelBody_N_e = cArrayToEigenVector3<double>(r_secCelBody_N.data);
    const Eigen::Vector3d v_secCelBody_N_e = cArrayToEigenVector3<double>(v_secCelBody_N.data);
    const Eigen::Vector3d r_BN_N_e = cArrayToEigenVector3<double>(r_BN_N.data);
    const Eigen::Vector3d v_BN_N_e = cArrayToEigenVector3<double>(v_BN_N.data);

    // clang-format off
    const CelestialTwoBodyPointOutput out = reinterpret_cast<const ::CelestialTwoBodyPointAlgorithm*>(self)->update(r_celBody_N_e, v_celBody_N_e, r_secCelBody_N_e, v_secCelBody_N_e, r_BN_N_e, v_BN_N_e);  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    // clang-format on

    AttRefMsgF32Payload payload{};
    eigenVectorToCArray(out.sigma_RN, payload.sigma_RN);
    eigenVectorToCArray(out.omega_RN_N, payload.omega_RN_N);
    eigenVectorToCArray(out.domega_RN_N, payload.domega_RN_N);
    return payload;
}
