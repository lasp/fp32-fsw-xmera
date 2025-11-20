/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
*/

#ifndef F32XIMERA_CELESTIAL_BODY_POINT_ALGORITHM_H
#define F32XIMERA_CELESTIAL_BODY_POINT_ALGORITHM_H

#include "msgPayloadDef/AttRefMsgF32Payload.h"
#include "msgPayloadDef/EphemerisMsgF32Payload.h"
#include "msgPayloadDef/NavTransMsgF32Payload.h"
#include <Eigen/Core>

/*!@brief Data structure for module to compute the two-body celestial pointing navigation solution.
 */
class CelestialTwoBodyPointAlgorithm {
   public:
    void reset(bool secCelBodyIsLinked);
    AttRefMsgF32Payload update(EphemerisMsgF32Payload& celBodyIn,
                               EphemerisMsgF32Payload& secCelBodyIn,
                               NavTransMsgF32Payload& transNavIn) const;
    void setSingularityThresh(float thresh);
    float getSingularityThresh() const;

   private:
    float singularityThresh{};  //!< [rad] Threshold for when to fix constraint axis*/
    bool secCelBodyIsLinked{};  //!< flag to indicate if the optional 2nd celestial body message is linked
};

#endif
