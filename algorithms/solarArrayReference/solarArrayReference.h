#ifndef F32XMERA_SOLAR_ARRAY_REFERENCE_H
#define F32XMERA_SOLAR_ARRAY_REFERENCE_H

#include "solarArrayReferenceAlgorithm.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/AttRefMsgPayload.h>
#include <architecture/msgPayloadDef/HingedRigidBodyMsgPayload.h>
#include <architecture/msgPayloadDef/NavAttMsgPayload.h>
#include <stdint.h>

/*! @brief adapter for the solar array reference algorithm. */
class SolarArrayReference : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    void setA1Hat_B(const Eigen::Vector3d& axis);
    Eigen::Vector3d getA1Hat_B() const;
    void setA2Hat_B(const Eigen::Vector3d& normal);
    Eigen::Vector3d getA2Hat_B() const;
    void setAttitudeFrame(int frame);
    int getAttitudeFrame() const;

    /* declare module IO interfaces */
    ReadFunctor<NavAttMsgPayload> attNavInMsg;                    //!< input msg measured attitude
    ReadFunctor<AttRefMsgPayload> attRefInMsg;                    //!< input attitude reference message
    ReadFunctor<HingedRigidBodyMsgPayload> hingedRigidBodyInMsg;  //!< input hinged rigid body message
    Message<HingedRigidBodyMsgPayload>
        hingedRigidBodyRefOutMsg;  //!< output msg containing hinged rigid body target angle and angle rate

   private:
    SolarArrayReferenceAlgorithm algorithm{};  //!< algorithm instance
};

#endif  // F32XMERA_SOLAR_ARRAY_REFERENCE_H
