#include "mrpPDAlgorithm_c.h"
#include "mrpPDAlgorithm.h"
#include "mrpPDTypes.h"
#include "utilities/fsw/eigenSupport.h"

#include <Eigen/Core>

namespace {
MrpPDConfig configFromC(const MrpPDConfig_c& c) {
    return MrpPDConfig::create(
        c.K, c.P, cArrayToEigenVector3<float>(c.knownTorquePntB_B.data), c2DArrayToEigenMatrix3(c.ISCPntB_B.data));
}
}  // namespace

MrpPDAlgorithmHandle* MrpPDAlgorithm_create(const MrpPDConfig_c* config) {
    return reinterpret_cast<MrpPDAlgorithmHandle*>(new ::MrpPDAlgorithm(configFromC(*config)));
}

void MrpPDAlgorithm_destroy(MrpPDAlgorithmHandle* self) { delete reinterpret_cast<::MrpPDAlgorithm*>(self); }

void MrpPDAlgorithm_setConfig(MrpPDAlgorithmHandle* self, const MrpPDConfig_c* config) {
    reinterpret_cast<::MrpPDAlgorithm*>(self)->setConfig(configFromC(*config));
}

Vector3f_c MrpPDAlgorithm_update(const MrpPDAlgorithmHandle* self,
                                 Vector3f_c sigma_BR,
                                 Vector3f_c omega_BR_B,
                                 Vector3f_c domega_RN_B) {
    const Eigen::Vector3f torque =
        reinterpret_cast<const ::MrpPDAlgorithm*>(self)->update(cArrayToEigenVector3<float>(sigma_BR.data),
                                                                cArrayToEigenVector3<float>(omega_BR_B.data),
                                                                cArrayToEigenVector3<float>(domega_RN_B.data));

    Vector3f_c out{};
    eigenVectorToCArray(torque, out.data);
    return out;
}
