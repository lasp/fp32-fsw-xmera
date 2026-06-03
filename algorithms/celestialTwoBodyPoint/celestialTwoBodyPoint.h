#ifndef F32XMERA_CELESTIAL_BODY_POINT_H
#define F32XMERA_CELESTIAL_BODY_POINT_H

#include <stdint.h>

#include "celestialTwoBodyPointAlgorithm.h"
#include "msgPayloadDef/AttRefMsgF32Payload.h"
#include "msgPayloadDef/EphemerisMsgF32Payload.h"
#include "msgPayloadDef/NavTransMsgF32Payload.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>

/*!@brief Celestial two-body pointing attitude guidance adapter.
 */
class CelestialTwoBodyPoint final : public SysModel {
   public:
    CelestialTwoBodyPoint() = default;
    ~CelestialTwoBodyPoint() = default;

    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;
    void setSingularityThreshold(float threshold);
    float getSingularityThreshold() const;
    void setRateThreshold(float rateThreshold);
    float getRateThreshold() const;

    Message<AttRefMsgF32Payload> attRefOutMsg;            //!< Attitude reference output message
    ReadFunctor<EphemerisMsgF32Payload> celBodyInMsg;     //!< Primary celestial body ephemeris input message
    ReadFunctor<EphemerisMsgF32Payload> secCelBodyInMsg;  //!< (optional) Secondary celestial body ephemeris input
    ReadFunctor<NavTransMsgF32Payload> transNavInMsg;     //!< Spacecraft translational navigation input message

   private:
    bool secCelBodyIsLinked{};
    CelestialTwoBodyPointAlgorithm algorithm{};
};

#endif
