#ifndef F32XMERA_SOLAR_ARRAY_REFERENCE_H
#define F32XMERA_SOLAR_ARRAY_REFERENCE_H

#include "msgPayloadDef/AttRefMsgF32Payload.h"
#include "msgPayloadDef/HingedRigidBodyMsgF32Payload.h"
#include "msgPayloadDef/MotorAngleRefMsgF32Payload.h"
#include "msgPayloadDef/NavAttMsgF32Payload.h"
#include "solarArrayReferenceAlgorithm.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <stdint.h>
#include <array>

/*! @brief adapter for the solar array reference algorithm. */
class SolarArrayReference : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    void setSolarArrayAxes_B(const Eigen::Vector3f& driveAxis, const Eigen::Vector3f& surfaceNormal);
    std::array<Eigen::Vector3f, 2> getSolarArrayAxes_B() const;
    void setAlignmentThreshold(float threshold);
    float getAlignmentThreshold() const;
    void setTrackingMode(TrackingMode mode);
    TrackingMode getTrackingMode() const;
    void setSpecifiedArrayAngle(float angle);
    float getSpecifiedArrayAngle() const;
    void setOffsetAngle(float angle);
    float getOffsetAngle() const;

    /* declare module IO interfaces */
    ReadFunctor<NavAttMsgF32Payload> attNavInMsg;                    //!< input msg measured attitude
    ReadFunctor<AttRefMsgF32Payload> attRefInMsg;                    //!< input attitude reference message
    ReadFunctor<HingedRigidBodyMsgF32Payload> hingedRigidBodyInMsg;  //!< input hinged rigid body message
    Message<MotorAngleRefMsgF32Payload> solarArrayRefOutMsg;  //!< output msg containing the solar array reference angle

   private:
    SolarArrayReferenceAlgorithm algorithm{};  //!< algorithm instance
};

#endif  // F32XMERA_SOLAR_ARRAY_REFERENCE_H
