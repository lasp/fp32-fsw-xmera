/* MIT License
 *
 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#include "bodyRateMiscompareAlgorithm_c.h"
#include "bodyRateMiscompareAlgorithm.h"

#include <Eigen/Core>

BodyRateMiscompareAlgorithm* BodyRateMiscompareAlgorithm_create(void) {
    return reinterpret_cast<BodyRateMiscompareAlgorithm*>(new ::BodyRateMiscompareAlgorithm());
}

void BodyRateMiscompareAlgorithm_destroy(BodyRateMiscompareAlgorithm* self) {
    delete reinterpret_cast<::BodyRateMiscompareAlgorithm*>(self);
}

BodyRateMiscompareOutput_c BodyRateMiscompareAlgorithm_update(const BodyRateMiscompareAlgorithm* self,
                                                              Vector3f_c imuOmega,
                                                              Vector3f_c stOmega) {
    Eigen::Vector3f imuVec;
    imuVec << imuOmega.data[0], imuOmega.data[1], imuOmega.data[2];
    Eigen::Vector3f stVec;
    stVec << stOmega.data[0], stOmega.data[1], stOmega.data[2];

    BodyRateMiscompareOutput result =
        reinterpret_cast<const ::BodyRateMiscompareAlgorithm*>(self)->update(imuVec, stVec);

    BodyRateMiscompareOutput_c out;
    out.omega_BN_B[0] = result.omega_BN_B[0];
    out.omega_BN_B[1] = result.omega_BN_B[1];
    out.omega_BN_B[2] = result.omega_BN_B[2];
    out.bodyRateFaultDetected = result.bodyRateFaultDetected;
    return out;
}

void BodyRateMiscompareAlgorithm_setBodyRateThreshold(BodyRateMiscompareAlgorithm* self, float bodyRateThreshold) {
    reinterpret_cast<::BodyRateMiscompareAlgorithm*>(self)->setBodyRateThreshold(bodyRateThreshold);
}

float BodyRateMiscompareAlgorithm_getBodyRateThreshold(const BodyRateMiscompareAlgorithm* self) {
    return reinterpret_cast<const ::BodyRateMiscompareAlgorithm*>(self)->getBodyRateThreshold();
}
