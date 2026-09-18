#include "mrpRotationAlgorithm_c.h"
#include "mrpRotationAlgorithm.h"
#include "mrpRotationTypes.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/opaqueHandle.h"

#include <Eigen/Core>

namespace {
MrpRotationConfig configFromC(const Vector3f_c& initialSigmaRR0,
                              const Vector3f_c& omegaRR0R,
                              const float controlPeriod) {
    return MrpRotationConfig::create(
        cArrayToEigenVector3<float>(initialSigmaRR0.data), cArrayToEigenVector3<float>(omegaRR0R.data), controlPeriod);
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

bool MrpRotationAlgorithm_validateConfig(const Vector3f_c* initialSigmaRR0,
                                         const Vector3f_c* omegaRR0R,
                                         const float controlPeriod) {
    // Build the config through the same path create() uses: success means valid, a throw means
    // invalid. Sharing configFromC keeps the predicate from drifting from what create() accepts.
    try {
        (void)configFromC(*initialSigmaRR0, *omegaRR0R, controlPeriod);
        return true;
    } catch (const fsw::invalid_argument&) {
        return false;
    }
}

MrpRotationAlgorithmHandle* MrpRotationAlgorithm_create(const Vector3f_c* initialSigmaRR0,
                                                        const Vector3f_c* omegaRR0R,
                                                        const float controlPeriod) {
    return fsw::createHandle<::MrpRotationAlgorithm, MrpRotationAlgorithmHandle>(
        configFromC(*initialSigmaRR0, *omegaRR0R, controlPeriod));
}

void MrpRotationAlgorithm_destroy(MrpRotationAlgorithmHandle* self) { fsw::deleteHandle<::MrpRotationAlgorithm>(self); }

void MrpRotationAlgorithm_setConfig(MrpRotationAlgorithmHandle* self,
                                    const Vector3f_c* initialSigmaRR0,
                                    const Vector3f_c* omegaRR0R,
                                    const float controlPeriod) {
    fsw::fromHandle<::MrpRotationAlgorithm>(self)->setConfig(configFromC(*initialSigmaRR0, *omegaRR0R, controlPeriod));
}

MrpRotationOutput_c MrpRotationAlgorithm_update(MrpRotationAlgorithmHandle* self,
                                                const MrpRotationAttRefInputs_c* attRef) {
    const MrpRotationOutput out = fsw::fromHandle<::MrpRotationAlgorithm>(self)->update(attRefFromC(*attRef));
    return outputToC(out);
}

void MrpRotationAlgorithm_reInitialize(MrpRotationAlgorithmHandle* self) {
    fsw::fromHandle<::MrpRotationAlgorithm>(self)->reInitialize();
}
