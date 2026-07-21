#include "bodyRateMiscompareAlgorithm_c.h"
#include "bodyRateMiscompareAlgorithm.h"
#include "bodyRateMiscompareTypes.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/opaqueHandle.h"

#include <Eigen/Core>

namespace {
BodyRateMiscompareConfig configFromC(const BodyRateMiscompareConfig_c& c) {
    return BodyRateMiscompareConfig::create(c.bodyRateThreshold, c.faultPersistenceLimit, c.useImuRates);
}
}  // namespace

BodyRateMiscompareAlgorithmHandle* BodyRateMiscompareAlgorithm_create(const BodyRateMiscompareConfig_c* config) {
    return fsw::createHandle<::BodyRateMiscompareAlgorithm, BodyRateMiscompareAlgorithmHandle>(configFromC(*config));
}

void BodyRateMiscompareAlgorithm_destroy(BodyRateMiscompareAlgorithmHandle* self) {
    fsw::deleteHandle<::BodyRateMiscompareAlgorithm>(self);
}

void BodyRateMiscompareAlgorithm_setConfig(BodyRateMiscompareAlgorithmHandle* self,
                                           const BodyRateMiscompareConfig_c* config) {
    fsw::fromHandle<::BodyRateMiscompareAlgorithm>(self)->setConfig(configFromC(*config));
}

void BodyRateMiscompareAlgorithm_reInitializeExceptPersistentStates(BodyRateMiscompareAlgorithmHandle* self) {
    fsw::fromHandle<::BodyRateMiscompareAlgorithm>(self)->reInitializeExceptPersistentStates();
}

void BodyRateMiscompareAlgorithm_reInitialize(BodyRateMiscompareAlgorithmHandle* self) {
    fsw::fromHandle<::BodyRateMiscompareAlgorithm>(self)->reInitialize();
}

BodyRateMiscompareOutput_c BodyRateMiscompareAlgorithm_update(BodyRateMiscompareAlgorithmHandle* self,
                                                              Vector3f_c imuOmega,
                                                              Vector3f_c stOmega) {
    const BodyRateMiscompareOutput result = fsw::fromHandle<::BodyRateMiscompareAlgorithm>(self)->update(
        cArrayToEigenVector3<float>(imuOmega.data), cArrayToEigenVector3<float>(stOmega.data));

    BodyRateMiscompareOutput_c out{};
    eigenVectorToCArray(result.omega_BN_B, out.omega_BN_B);
    out.bodyRateFaultDetected = result.bodyRateFaultDetected;
    return out;
}
