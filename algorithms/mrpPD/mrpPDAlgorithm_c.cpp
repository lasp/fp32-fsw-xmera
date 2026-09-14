#include "mrpPDAlgorithm_c.h"
#include "mrpPDAlgorithm.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/opaqueHandle.h"

#include <Eigen/Core>

namespace {
MrpPDConfig configFromC(float K, float P, const Vector3f_c& knownTorquePntB_B, const Matrix3f_c& ISCPntB_B) {
    return MrpPDConfig::create(
        K, P, cArrayToEigenVector3<float>(knownTorquePntB_B.data), c2DArrayToEigenMatrix3(ISCPntB_B.data));
}
}  // namespace

bool MrpPDAlgorithm_validateConfig(const float K,
                                   const float P,
                                   const Vector3f_c knownTorquePntB_B,
                                   const Matrix3f_c ISCPntB_B) {
    // Attempt to build the config through the real create path; success means valid,
    // a throw means invalid. Reusing configFromC keeps validation from drifting.
    try {
        (void)configFromC(K, P, knownTorquePntB_B, ISCPntB_B);
        return true;
    } catch (const fsw::invalid_argument&) {
        return false;
    }
}

MrpPDAlgorithmHandle* MrpPDAlgorithm_create(const float K,
                                            const float P,
                                            const Vector3f_c knownTorquePntB_B,
                                            const Matrix3f_c ISCPntB_B) {
    return fsw::createHandle<::MrpPDAlgorithm, MrpPDAlgorithmHandle>(configFromC(K, P, knownTorquePntB_B, ISCPntB_B));
}

void MrpPDAlgorithm_destroy(MrpPDAlgorithmHandle* self) { fsw::deleteHandle<::MrpPDAlgorithm>(self); }

void MrpPDAlgorithm_setConfig(MrpPDAlgorithmHandle* self,
                              const float K,
                              const float P,
                              const Vector3f_c knownTorquePntB_B,
                              const Matrix3f_c ISCPntB_B) {
    fsw::fromHandle<::MrpPDAlgorithm>(self)->setConfig(configFromC(K, P, knownTorquePntB_B, ISCPntB_B));
}

Vector3f_c MrpPDAlgorithm_update(const MrpPDAlgorithmHandle* self,
                                 const Vector3f_c sigma_BR,
                                 const Vector3f_c omega_BR_B,
                                 const Vector3f_c domega_RN_B) {
    const Eigen::Vector3f torque =
        fsw::fromHandle<const ::MrpPDAlgorithm>(self)->update(cArrayToEigenVector3<float>(sigma_BR.data),
                                                              cArrayToEigenVector3<float>(omega_BR_B.data),
                                                              cArrayToEigenVector3<float>(domega_RN_B.data));

    Vector3f_c out{};
    eigenVectorToCArray(torque, out.data);
    return out;
}
