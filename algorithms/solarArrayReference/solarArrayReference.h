#ifndef F32XMERA_SOLAR_ARRAY_REFERENCE_H
#define F32XMERA_SOLAR_ARRAY_REFERENCE_H

#include "msgPayloadDef/AttRefMsgF32Payload.h"
#include "msgPayloadDef/HingedRigidBodyMsgF32Payload.h"
#include "msgPayloadDef/NavAttMsgF32Payload.h"
#include "solarArrayReferenceAlgorithm.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <stdint.h>

/*! @brief adapter for the solar array reference algorithm. */
class SolarArrayReference : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    void setA1Hat_B(const Eigen::Vector3f& axis);
    Eigen::Vector3f getA1Hat_B() const;
    void setA2Hat_B(const Eigen::Vector3f& normal);
    Eigen::Vector3f getA2Hat_B() const;
    /* declare module IO interfaces */
    ReadFunctor<NavAttMsgF32Payload> attNavInMsg;                    //!< input msg measured attitude
    ReadFunctor<AttRefMsgF32Payload> attRefInMsg;                    //!< input attitude reference message
    ReadFunctor<HingedRigidBodyMsgF32Payload> hingedRigidBodyInMsg;  //!< input hinged rigid body message
    Message<HingedRigidBodyMsgF32Payload>
        hingedRigidBodyRefOutMsg;  //!< output msg containing hinged rigid body target angle and angle rate

   private:
    SolarArrayReferenceAlgorithm algorithm{};  //!< algorithm instance
};

#endif  // F32XMERA_SOLAR_ARRAY_REFERENCE_H
