#ifndef F32XMERA_CELESTIAL_BODY_POINT_H
#define F32XMERA_CELESTIAL_BODY_POINT_H

#include <stdint.h>

#include "celestialTwoBodyPointAlgorithm.h"
#include "msgPayloadDef/AttRefMsgF32Payload.h"
#include "msgPayloadDef/EphemerisMsgF32Payload.h"
#include "msgPayloadDef/NavTransMsgF32Payload.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <memory>

/*!@brief Celestial two-body pointing attitude guidance adapter.
 */
class CelestialTwoBodyPoint final : public SysModel {
   public:
    CelestialTwoBodyPoint() = default;
    ~CelestialTwoBodyPoint() = default;

    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;
    void reconfigure() const;
    void reInitialize();
    void reInitializeAll();

    // Phase 1: Public config properties — set before reset()
    float celestialBodyAlignmentThreshold =
        1e-3F;  //!< [rad] Angle threshold for primary and secondary celestial body alignment check

    Message<AttRefMsgF32Payload> attRefOutMsg;                  //!< Attitude reference output message
    ReadFunctor<EphemerisMsgF32Payload> primaryCelBodyInMsg;    //!< Primary celestial body ephemeris input message
    ReadFunctor<EphemerisMsgF32Payload> secondaryCelBodyInMsg;  //!< Secondary celestial body ephemeris input message
    ReadFunctor<NavTransMsgF32Payload> transNavInMsg;           //!< Spacecraft translational navigation input message

   private:
    std::unique_ptr<CelestialTwoBodyPointAlgorithm> algorithm = nullptr;
    CelestialTwoBodyPointConfig toConfig() const;
};

#endif
