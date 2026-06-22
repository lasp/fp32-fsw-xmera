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
#include <Eigen/Core>
#include <memory>

/*! @brief adapter for the solar array reference algorithm. */
class SolarArrayReference final : public SysModel {
   public:
    SolarArrayReference() = default;
    ~SolarArrayReference() override = default;

    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    // Phase 1: public config properties -- set before reset().
    Eigen::Vector3f driveAxis = Eigen::Vector3f::Zero();      //!< [-] solar array drive axis in body frame
    Eigen::Vector3f surfaceNormal = Eigen::Vector3f::Zero();  //!< [-] solar array surface normal at zero rotation
    float alignmentThreshold = 1e-3F;  //!< [rad] alignment threshold angle between sun direction and drive axis
    TrackingMode trackingMode = TrackingMode::AUTO_TRACK;  //!< array tracking mode
    float specifiedArrayAngle{};  //!< [rad] specified reference array angle when tracking mode is specified angle
    float offsetAngle{};          //!< [rad] offset angle added to the determined reference angle

    /* declare module IO interfaces */
    ReadFunctor<NavAttMsgF32Payload> attNavInMsg;                    //!< input msg measured attitude
    ReadFunctor<AttRefMsgF32Payload> attRefInMsg;                    //!< input attitude reference message
    ReadFunctor<HingedRigidBodyMsgF32Payload> hingedRigidBodyInMsg;  //!< input hinged rigid body message
    Message<MotorAngleRefMsgF32Payload> solarArrayRefOutMsg;  //!< output msg containing the solar array reference angle

   private:
    std::unique_ptr<SolarArrayReferenceAlgorithm> algorithm = nullptr;  //!< algorithm instance
};

#endif  // F32XMERA_SOLAR_ARRAY_REFERENCE_H
