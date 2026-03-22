#ifndef F32XMERA_ATT_TRACKING_ERROR_ALGORITHM_H
#define F32XMERA_ATT_TRACKING_ERROR_ALGORITHM_H

#include "attTrackingErrorTypes.h"

class AttTrackingErrorAlgorithm {
   public:
    static AttGuidOutput update(const AttNavInput& navIn, const AttRefInput& refIn);  //!< Algorithm update method
};

#endif
