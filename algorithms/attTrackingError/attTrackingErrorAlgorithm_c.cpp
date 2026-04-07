/* MIT License
 *
 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#include "attTrackingErrorAlgorithm_c.h"
#include "attTrackingErrorAlgorithm.h"

#include "architecture/utilities/eigenSupport.h"
#include <Eigen/Core>

AttTrackingErrorAlgorithm* AttTrackingErrorAlgorithm_create(void) {
    return reinterpret_cast<AttTrackingErrorAlgorithm*>(new ::AttTrackingErrorAlgorithm());
}

void AttTrackingErrorAlgorithm_destroy(AttTrackingErrorAlgorithm* self) {
    delete reinterpret_cast<::AttTrackingErrorAlgorithm*>(self);
}

AttGuidMsgF32Payload AttTrackingErrorAlgorithm_update(AttTrackingErrorAlgorithm* self,
                                                      AttRefMsgF32Payload* attRefInMsg,
                                                      NavAttMsgF32Payload* attNavInMsg) {
    return reinterpret_cast<::AttTrackingErrorAlgorithm*>(self)->update(*attRefInMsg, *attNavInMsg);
}
