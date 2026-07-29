#include "celestialTwoBodyPointAlgorithm_c.h"
#include "celestialTwoBodyPointAlgorithm.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/opaqueHandle.h"

#include <Eigen/Core>

bool CelestialTwoBodyPointAlgorithm_validateConfig(float celestialBodyAlignmentThreshold) {
    try {
        (void)CelestialTwoBodyPointConfig::create(celestialBodyAlignmentThreshold);
        return true;
    } catch (const fsw::invalid_argument&) {
        return false;
    }
}

CelestialTwoBodyPointAlgorithmHandle* CelestialTwoBodyPointAlgorithm_create(float celestialBodyAlignmentThreshold) {
    return fsw::createHandle<::CelestialTwoBodyPointAlgorithm, CelestialTwoBodyPointAlgorithmHandle>(
        CelestialTwoBodyPointConfig::create(celestialBodyAlignmentThreshold));
}

void CelestialTwoBodyPointAlgorithm_destroy(CelestialTwoBodyPointAlgorithmHandle* self) {
    fsw::deleteHandle<::CelestialTwoBodyPointAlgorithm>(self);
}

void CelestialTwoBodyPointAlgorithm_setConfig(CelestialTwoBodyPointAlgorithmHandle* self,
                                              float celestialBodyAlignmentThreshold) {
    fsw::fromHandle<::CelestialTwoBodyPointAlgorithm>(self)->setConfig(
        CelestialTwoBodyPointConfig::create(celestialBodyAlignmentThreshold));
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

    const ::InertialStateInput primaryBodyState{r_PN_N_e, v_PN_N_e};
    const ::InertialStateInput secondaryBodyState{r_SN_N_e, v_SN_N_e};
    const ::InertialStateInput spacecraftState{r_BN_N_e, v_BN_N_e};

    const CelestialTwoBodyPointOutput out = fsw::fromHandle<const ::CelestialTwoBodyPointAlgorithm>(self)->update(
        primaryBodyState, secondaryBodyState, spacecraftState);

    AttRefMsgF32Payload payload{};
    eigenVectorToCArray(out.sigma_RN, payload.sigma_RN);
    eigenVectorToCArray(out.omega_RN_N, payload.omega_RN_N);
    eigenVectorToCArray(out.domega_RN_N, payload.domega_RN_N);
    return payload;
}
