#include "sunlineEphemAlgorithm_c.h"
#include "sunlineEphemAlgorithm.h"
#include "utilities/fsw/opaqueHandle.h"

#include <Eigen/Core>

SunlineEphemAlgorithmHandle* SunlineEphemAlgorithm_create(void) {
    return fsw::createHandle<::SunlineEphemAlgorithm, SunlineEphemAlgorithmHandle>();
}

void SunlineEphemAlgorithm_destroy(SunlineEphemAlgorithmHandle* self) {
    fsw::deleteHandle<::SunlineEphemAlgorithm>(self);
}

void SunlineEphemAlgorithm_update(const SunlineEphemAlgorithmHandle* self,
                                  const Vector3d_c* sunPos,
                                  const Vector3d_c* scPos,
                                  const Vector3f_c* sigmaBN,
                                  Vector3f_c* result) {
    Eigen::Vector3d r_SN_N;
    r_SN_N << sunPos->data[0], sunPos->data[1], sunPos->data[2];

    Eigen::Vector3d r_BN_N;
    r_BN_N << scPos->data[0], scPos->data[1], scPos->data[2];

    Eigen::Vector3f sigma_BN;
    sigma_BN << sigmaBN->data[0], sigmaBN->data[1], sigmaBN->data[2];

    const Eigen::Vector3f rHat_SB_B =
        fsw::fromHandle<const ::SunlineEphemAlgorithm>(self)->update(r_SN_N, r_BN_N, sigma_BN);

    result->data[0] = rHat_SB_B[0];
    result->data[1] = rHat_SB_B[1];
    result->data[2] = rHat_SB_B[2];
}
