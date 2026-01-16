#ifndef F32XMERA_CELESTIAL_BODY_POINT_ALGORITHM_H
#define F32XMERA_CELESTIAL_BODY_POINT_ALGORITHM_H

#include "msgPayloadDef/AttRefMsgF32Payload.h"
#include "msgPayloadDef/EphemerisMsgF32Payload.h"
#include "msgPayloadDef/NavTransMsgF32Payload.h"
#include <Eigen/Core>

/*!@brief Data structure for module to compute the two-body celestial pointing navigation solution.
 */
class CelestialTwoBodyPointAlgorithm final {
   public:
    void reset(bool secCelBodyIsLinkedIn);
    AttRefMsgF32Payload update(EphemerisMsgF32Payload& celBodyIn,
                               EphemerisMsgF32Payload& secCelBodyIn,
                               NavTransMsgF32Payload& transNavIn) const;

    static AttRefMsgF32Payload rateAndAccelCalc(const Eigen::Vector3d &r_PB_N, const Eigen::Vector3d &v_PB_N,
                                                const Eigen::Vector3d &r_SB_N, const Eigen::Vector3d &v_SB_N);
    void setSingularityThreshold(float singularityThresholdIn);
    float getSingularityThreshold() const;

   private:
    float singularityThreshold{};  //!< [rad] Threshold for when to fix constraint axis*/
    float rateThreshold{};      //!< [rad/s] Rate threshold for when to fix constraint axis
    bool secCelBodyIsLinked{};  //!< flag to indicate if the optional 2nd celestial body message is linked
};

#endif
