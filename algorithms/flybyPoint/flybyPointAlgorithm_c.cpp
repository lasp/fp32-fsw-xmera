#include "flybyPointAlgorithm_c.h"
#include "flybyPointAlgorithm.h"
#include "flybyPointTypes.h"
#include "utilities/fsw/eigenSupport.h"

#include <Eigen/Core>

namespace {
FlybyPointConfig configFromC(const FlybyPointConfig_c& c) {
    return FlybyPointConfig::create(c.controlPeriod,
                                    c.filterReadPeriods,
                                    c.toleranceForCollinearity,
                                    c.signOfOrbitNormalFrameVector,
                                    c.maximumRateThreshold,
                                    c.maximumAccelerationThreshold,
                                    c.numberOfSigmas,
                                    cArrayToEigenVector3<float>(c.groundPosition_BN_N.data),
                                    cArrayToEigenVector3<float>(c.groundVelocity_BN_N.data),
                                    cArrayToEigenVector3<float>(c.groundPositionCovarianceDiag_N.data),
                                    cArrayToEigenVector3<float>(c.groundVelocityCovarianceDiag_N.data));
}
}  // namespace

FlybyPointAlgorithmHandle* FlybyPointAlgorithm_create(const FlybyPointConfig_c* config) {
    return reinterpret_cast<FlybyPointAlgorithmHandle*>(new ::FlybyPointAlgorithm(configFromC(*config)));
}

void FlybyPointAlgorithm_destroy(FlybyPointAlgorithmHandle* self) {
    delete reinterpret_cast<::FlybyPointAlgorithm*>(self);
}

void FlybyPointAlgorithm_setConfig(FlybyPointAlgorithmHandle* self, const FlybyPointConfig_c* config) {
    reinterpret_cast<::FlybyPointAlgorithm*>(self)->setConfig(configFromC(*config));
}

void FlybyPointAlgorithm_reset(FlybyPointAlgorithmHandle* self) {
    reinterpret_cast<::FlybyPointAlgorithm*>(self)->reset();
}

AttGuideOutput_c FlybyPointAlgorithm_updateState(FlybyPointAlgorithmHandle* self,
                                                 const Vector3f_c r_BN_N,
                                                 const Vector3f_c v_BN_N) {
    const Eigen::Vector3f r_BN_N_e = cArrayToEigenVector3<float>(r_BN_N.data);
    const Eigen::Vector3f v_BN_N_e = cArrayToEigenVector3<float>(v_BN_N.data);

    const AttGuideOutput out = reinterpret_cast<::FlybyPointAlgorithm*>(self)->updateState(r_BN_N_e, v_BN_N_e);

    AttGuideOutput_c result{};
    eigenVectorToCArray(out.sigma_RN, result.sigma_RN.data);
    eigenVectorToCArray(out.omega_RN_N, result.omega_RN_N.data);
    eigenVectorToCArray(out.domega_RN_N, result.domega_RN_N.data);
    result.collinearityTrigger = out.collinearityTrigger;
    result.maxRateTrigger = out.maxRateTrigger;
    result.maxAccelerationTrigger = out.maxAccelerationTrigger;
    result.stateKnowledgeExceedTrigger = out.stateKnowledgeExceedTrigger;
    result.inputSampleRejected = out.inputSampleRejected;
    result.rejectedSamplesInWindow = out.rejectedSamplesInWindow;
    return result;
}
