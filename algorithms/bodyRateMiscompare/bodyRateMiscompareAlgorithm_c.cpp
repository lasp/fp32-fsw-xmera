#include "bodyRateMiscompareAlgorithm_c.h"
#include "bodyRateMiscompareAlgorithm.h"

#include <Eigen/Core>

BodyRateMiscompareAlgorithmHandle* BodyRateMiscompareAlgorithm_create(void) {
    return reinterpret_cast<BodyRateMiscompareAlgorithmHandle*>(new ::BodyRateMiscompareAlgorithm());
}

void BodyRateMiscompareAlgorithm_destroy(BodyRateMiscompareAlgorithmHandle* self) {
    delete reinterpret_cast<::BodyRateMiscompareAlgorithm*>(self);
}

void BodyRateMiscompareAlgorithm_reset(BodyRateMiscompareAlgorithmHandle* self) {
    reinterpret_cast<::BodyRateMiscompareAlgorithm*>(self)->reset();
}

BodyRateMiscompareOutput_c BodyRateMiscompareAlgorithm_update(BodyRateMiscompareAlgorithmHandle* self,
                                                              Vector3f_c imuOmega,
                                                              Vector3f_c stOmega) {
    Eigen::Vector3f imuVec;
    imuVec << imuOmega.data[0], imuOmega.data[1], imuOmega.data[2];
    Eigen::Vector3f stVec;
    stVec << stOmega.data[0], stOmega.data[1], stOmega.data[2];

    BodyRateMiscompareOutput result = reinterpret_cast<::BodyRateMiscompareAlgorithm*>(self)->update(imuVec, stVec);

    BodyRateMiscompareOutput_c out;
    out.omega_BN_B[0] = result.omega_BN_B[0];
    out.omega_BN_B[1] = result.omega_BN_B[1];
    out.omega_BN_B[2] = result.omega_BN_B[2];
    out.bodyRateFaultDetected = result.bodyRateFaultDetected;
    return out;
}

void BodyRateMiscompareAlgorithm_setBodyRateThreshold(BodyRateMiscompareAlgorithmHandle* self,
                                                      float bodyRateThreshold) {
    reinterpret_cast<::BodyRateMiscompareAlgorithm*>(self)->setBodyRateThreshold(bodyRateThreshold);
}

float BodyRateMiscompareAlgorithm_getBodyRateThreshold(const BodyRateMiscompareAlgorithmHandle* self) {
    return reinterpret_cast<const ::BodyRateMiscompareAlgorithm*>(self)->getBodyRateThreshold();
}

void BodyRateMiscompareAlgorithm_setFaultPersistenceLimit(BodyRateMiscompareAlgorithmHandle* self,
                                                          uint32_t faultPersistenceLimit) {
    reinterpret_cast<::BodyRateMiscompareAlgorithm*>(self)->setFaultPersistenceLimit(faultPersistenceLimit);
}

uint32_t BodyRateMiscompareAlgorithm_getFaultPersistenceLimit(const BodyRateMiscompareAlgorithmHandle* self) {
    return reinterpret_cast<const ::BodyRateMiscompareAlgorithm*>(self)->getFaultPersistenceLimit();
}

void BodyRateMiscompareAlgorithm_setUseImuRates(BodyRateMiscompareAlgorithmHandle* self, bool useImuRates) {
    reinterpret_cast<::BodyRateMiscompareAlgorithm*>(self)->setUseImuRates(useImuRates);
}

bool BodyRateMiscompareAlgorithm_getUseImuRates(const BodyRateMiscompareAlgorithmHandle* self) {
    return reinterpret_cast<const ::BodyRateMiscompareAlgorithm*>(self)->getUseImuRates();
}
