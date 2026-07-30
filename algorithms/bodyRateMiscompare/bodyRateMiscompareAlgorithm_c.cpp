#include "bodyRateMiscompareAlgorithm_c.h"
#include "bodyRateMiscompareAlgorithm.h"
#include "bodyRateMiscompareTypes.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/opaqueHandle.h"

#include <Eigen/Core>

namespace {
BodyRateMiscompareConfig configFromC(const float bodyRateThreshold,
                                     const uint32_t faultPersistenceLimit,
                                     const bool useImuRates) {
    return BodyRateMiscompareConfig::create(bodyRateThreshold, faultPersistenceLimit, useImuRates);
}
}  // namespace

bool BodyRateMiscompareAlgorithm_validateConfig(const float bodyRateThreshold,
                                                const uint32_t faultPersistenceLimit,
                                                const bool useImuRates) {
    // Attempt to build the config through the real create path; success means valid,
    // a throw means invalid. Reusing configFromC keeps validation from drifting.
    try {
        (void)configFromC(bodyRateThreshold, faultPersistenceLimit, useImuRates);
        return true;
    } catch (const fsw::invalid_argument&) {
        return false;
    }
}

BodyRateMiscompareAlgorithmHandle* BodyRateMiscompareAlgorithm_create(const float bodyRateThreshold,
                                                                      const uint32_t faultPersistenceLimit,
                                                                      const bool useImuRates) {
    return fsw::createHandle<::BodyRateMiscompareAlgorithm, BodyRateMiscompareAlgorithmHandle>(
        configFromC(bodyRateThreshold, faultPersistenceLimit, useImuRates));
}

void BodyRateMiscompareAlgorithm_destroy(BodyRateMiscompareAlgorithmHandle* self) {
    fsw::deleteHandle<::BodyRateMiscompareAlgorithm>(self);
}

void BodyRateMiscompareAlgorithm_setConfig(BodyRateMiscompareAlgorithmHandle* self,
                                           const float bodyRateThreshold,
                                           const uint32_t faultPersistenceLimit,
                                           const bool useImuRates) {
    fsw::fromHandle<::BodyRateMiscompareAlgorithm>(self)->setConfig(
        configFromC(bodyRateThreshold, faultPersistenceLimit, useImuRates));
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
