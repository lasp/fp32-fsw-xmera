/* MIT License
 *
 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#include "attTrackingErrorAlgorithm_c.h"
#include "attTrackingErrorAlgorithm.h"

#include <Eigen/Core>

AttTrackingErrorAlgorithm* AttTrackingErrorAlgorithm_create(void) {
    return reinterpret_cast<AttTrackingErrorAlgorithm*>(new ::AttTrackingErrorAlgorithm());
}

void AttTrackingErrorAlgorithm_destroy(AttTrackingErrorAlgorithm* self) {
    delete reinterpret_cast<::AttTrackingErrorAlgorithm*>(self);
}

void AttTrackingErrorAlgorithm_reset(AttTrackingErrorAlgorithm* self, uint64_t callTime) {
    reinterpret_cast<::AttTrackingErrorAlgorithm*>(self)->reset(callTime);
}

AttGuidMsgF32Payload AttTrackingErrorAlgorithm_update(AttTrackingErrorAlgorithm* self,
                                                      uint64_t callTime,
                                                      AttRefMsgF32Payload* attRefInMsg,
                                                      NavAttMsgF32Payload* attNavInMsg) {
    return reinterpret_cast<::AttTrackingErrorAlgorithm*>(self)->update(callTime, *attRefInMsg, *attNavInMsg);
}

void AttTrackingErrorAlgorithm_setSigma_R0R(AttTrackingErrorAlgorithm* self, Vector3f_c sigma_R0R) {
    Eigen::Vector3f vec;
    vec << sigma_R0R.data[0], sigma_R0R.data[1], sigma_R0R.data[2];
    reinterpret_cast<::AttTrackingErrorAlgorithm*>(self)->setSigma_R0R(vec);
}

Vector3f_c AttTrackingErrorAlgorithm_getSigma_R0R(AttTrackingErrorAlgorithm* self) {
    Eigen::Vector3f vec = reinterpret_cast<::AttTrackingErrorAlgorithm*>(self)->getSigma_R0R();
    Vector3f_c out;
    out.data[0] = vec[0];
    out.data[1] = vec[1];
    out.data[2] = vec[2];
    return out;
}
