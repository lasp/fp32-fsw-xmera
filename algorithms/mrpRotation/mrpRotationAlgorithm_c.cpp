#include "mrpRotationAlgorithm_c.h"
#include "mrpRotationAlgorithm.h"
#include "mrpRotationTypes.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/opaqueHandle.h"

#include <Eigen/Core>

namespace {
MrpRotationConfig configFromC(const MrpRotationConfig_c& c) {
    return MrpRotationConfig::create(cArrayToEigenVector3<float>(c.initialSigmaRR0.data),
                                     cArrayToEigenVector3<float>(c.omegaRR0R.data),
                                     c.controlPeriod);
}

MrpRotationAttRefInputs attRefFromC(const MrpRotationAttRefInputs_c& c) {
    return MrpRotationAttRefInputs{
        cArrayToEigenVector3<float>(c.sigma_R0N.data),
        cArrayToEigenVector3<float>(c.omega_R0N_N.data),
        cArrayToEigenVector3<float>(c.domega_R0N_N.data),
    };
}

MrpRotationOutput_c outputToC(const MrpRotationOutput& out) {
    MrpRotationOutput_c result{};
    eigenVectorToCArray(out.sigma_RN, result.sigma_RN.data);
    eigenVectorToCArray(out.omega_RN_N, result.omega_RN_N.data);
    eigenVectorToCArray(out.domega_RN_N, result.domega_RN_N.data);
    return result;
}
}  // namespace

MrpRotationAlgorithmHandle* MrpRotationAlgorithm_create(const MrpRotationConfig_c* config) {
    return fsw::createHandle<::MrpRotationAlgorithm, MrpRotationAlgorithmHandle>(configFromC(*config));
}

void MrpRotationAlgorithm_destroy(MrpRotationAlgorithmHandle* self) { fsw::deleteHandle<::MrpRotationAlgorithm>(self); }

void MrpRotationAlgorithm_setConfig(MrpRotationAlgorithmHandle* self, const MrpRotationConfig_c* config) {
    fsw::fromHandle<::MrpRotationAlgorithm>(self)->setConfig(configFromC(*config));
}

MrpRotationOutput_c MrpRotationAlgorithm_update(MrpRotationAlgorithmHandle* self,
                                                const MrpRotationAttRefInputs_c* attRef) {
    const MrpRotationOutput out = fsw::fromHandle<::MrpRotationAlgorithm>(self)->update(attRefFromC(*attRef));
    return outputToC(out);
}

void MrpRotationAlgorithm_reInitialize(MrpRotationAlgorithmHandle* self) {
    fsw::fromHandle<::MrpRotationAlgorithm>(self)->reInitialize();
}
