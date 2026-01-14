#ifndef F32XMERA_CELESTIAL_BODY_POINT_H
#define F32XMERA_CELESTIAL_BODY_POINT_H

#include <stdint.h>

#include "celestialTwoBodyPointAlgorithm.h"
#include "msgPayloadDef/AttRefMsgF32Payload.h"
#include "msgPayloadDef/EphemerisMsgF32Payload.h"
#include "msgPayloadDef/NavTransMsgF32Payload.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>

/*!@brief Data structure for module to compute the two-body celestial pointing navigation solution.
 */
class CelestialTwoBodyPoint final : public SysModel {
   public:
    CelestialTwoBodyPoint() = default;
    ~CelestialTwoBodyPoint() = default;

    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;
    void setSingularityThresh(float thresh);
    float getSingularityThresh() const;

    Message<AttRefMsgF32Payload> attRefOutMsg;            //!< The name of the output message*/
    ReadFunctor<EphemerisMsgF32Payload> celBodyInMsg;     //!< The name of the celestial body message*/
    ReadFunctor<EphemerisMsgF32Payload> secCelBodyInMsg;  //!< The name of the secondary body to constrain point*/
    ReadFunctor<NavTransMsgF32Payload> transNavInMsg;     //!< The name of the incoming attitude command*/

   private:
    bool secCelBodyIsLinked{};
    CelestialTwoBodyPointAlgorithm algorithm{};
};

#endif
