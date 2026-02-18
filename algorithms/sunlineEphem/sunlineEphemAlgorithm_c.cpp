#include "sunlineEphemAlgorithm_c.h"
#include "sunlineEphemAlgorithm.h"

#include <Eigen/Core>

SunlineEphemAlgorithm* SunlineEphemAlgorithm_create(void) {
    return reinterpret_cast<SunlineEphemAlgorithm*>(new ::SunlineEphemAlgorithm());
}

void SunlineEphemAlgorithm_destroy(SunlineEphemAlgorithm* self) {
    delete reinterpret_cast<::SunlineEphemAlgorithm*>(self);
}

NavAttMsgF32Payload SunlineEphemAlgorithm_updateState(const SunlineEphemAlgorithm* self,
                                                      const EphemerisMsgF32Payload* sunPos,
                                                      const NavTransMsgF32Payload* scPos,
                                                      const NavAttMsgF32Payload* scAtt) {
    Eigen::Vector3d r_SN_N;
    r_SN_N << sunPos->r_BdyZero_N[0], sunPos->r_BdyZero_N[1], sunPos->r_BdyZero_N[2];

    Eigen::Vector3d r_BN_N;
    r_BN_N << scPos->r_BN_N[0], scPos->r_BN_N[1], scPos->r_BN_N[2];

    Eigen::Vector3f sigma_BN;
    sigma_BN << scAtt->sigma_BN[0], scAtt->sigma_BN[1], scAtt->sigma_BN[2];

    const Eigen::Vector3f rHat_SB_B =
        reinterpret_cast<const ::SunlineEphemAlgorithm*>(self)->updateState(r_SN_N, r_BN_N, sigma_BN);

    NavAttMsgF32Payload out{};
    out.vehSunPntBdy[0] = rHat_SB_B[0];
    out.vehSunPntBdy[1] = rHat_SB_B[1];
    out.vehSunPntBdy[2] = rHat_SB_B[2];
    return out;
}
