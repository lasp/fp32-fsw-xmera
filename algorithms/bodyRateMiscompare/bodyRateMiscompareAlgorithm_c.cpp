#include "bodyRateMiscompareAlgorithm_c.h"
#include "bodyRateMiscompareAlgorithm.h"
#include "bodyRateMiscompareTypes.h"
#include "utilities/fsw/eigenSupport.h"

#include <Eigen/Core>

namespace {
BodyRateMiscompareConfig configFromC(const BodyRateMiscompareConfig_c& c) {
    return BodyRateMiscompareConfig::create(c.bodyRateThreshold, c.faultPersistenceLimit, c.useImuRates);
}
}  // namespace

BodyRateMiscompareAlgorithmHandle* BodyRateMiscompareAlgorithm_create(const BodyRateMiscompareConfig_c* config) {
    return reinterpret_cast<BodyRateMiscompareAlgorithmHandle*>(
        new ::BodyRateMiscompareAlgorithm(configFromC(*config)));
}

void BodyRateMiscompareAlgorithm_destroy(BodyRateMiscompareAlgorithmHandle* self) {
    delete reinterpret_cast<::BodyRateMiscompareAlgorithm*>(self);
}

void BodyRateMiscompareAlgorithm_setConfig(BodyRateMiscompareAlgorithmHandle* self,
                                           const BodyRateMiscompareConfig_c* config) {
    reinterpret_cast<::BodyRateMiscompareAlgorithm*>(self)->setConfig(configFromC(*config));
}

void BodyRateMiscompareAlgorithm_reInitialize(BodyRateMiscompareAlgorithmHandle* self) {
    reinterpret_cast<::BodyRateMiscompareAlgorithm*>(self)->reInitialize();
}

void BodyRateMiscompareAlgorithm_reInitializeAll(BodyRateMiscompareAlgorithmHandle* self) {
    reinterpret_cast<::BodyRateMiscompareAlgorithm*>(self)->reInitializeAll();
}

BodyRateMiscompareOutput_c BodyRateMiscompareAlgorithm_update(BodyRateMiscompareAlgorithmHandle* self,
                                                              Vector3f_c imuOmega,
                                                              Vector3f_c stOmega) {
    const BodyRateMiscompareOutput result = reinterpret_cast<::BodyRateMiscompareAlgorithm*>(self)->update(
        cArrayToEigenVector3<float>(imuOmega.data), cArrayToEigenVector3<float>(stOmega.data));

    BodyRateMiscompareOutput_c out{};
    eigenVectorToCArray(result.omega_BN_B, out.omega_BN_B);
    out.bodyRateFaultDetected = result.bodyRateFaultDetected;
    return out;
}
