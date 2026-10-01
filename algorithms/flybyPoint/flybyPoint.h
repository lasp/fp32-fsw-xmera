#ifndef F32XMERA_FLYBY_POINT_H
#define F32XMERA_FLYBY_POINT_H

#include "flybyPointAlgorithm.h"
#include "msgPayloadDef/AttRefMsgF32Payload.h"
#include "msgPayloadDef/FlybyDiagnosticMsgF32Payload.h"
#include "msgPayloadDef/NavTransMsgF32Payload.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <Eigen/Dense>
#include <cstdint>
#include <memory>

/*! @brief A class to perform flyby pointing */
class FlybyPoint : public SysModel {
   public:
    void reset(uint64_t currentSimNanos) override;
    void updateState(uint64_t currentSimNanos) override;
    std::tuple<Eigen::Vector3d, Eigen::Vector3d> readRelativeState();

    // Phase 1: Public config properties — set before reset()
    double controlPeriod = 0.0;       //!< [s] time between updateState() calls; must match the task rate (> 0)
    uint32_t filterReadPeriods = 1U;  //!< [-] control periods between two consecutive filter re-reads (>= 1)
    float toleranceForCollinearity = 0.0F;
    int signOfOrbitNormalFrameVector = 1;
    float maximumRateThreshold = 0.0F;
    float maximumAccelerationThreshold = 0.0F;
    float positionKnowledgeSigma = 0.0F;

    ReadFunctor<NavTransMsgF32Payload> filterInMsg;               //!< input msg relative position w.r.t. asteroid
    Message<AttRefMsgF32Payload> attRefOutMsg;                    //!< Attitude reference output message
    Message<FlybyDiagnosticMsgF32Payload> flybyDiagnosticOutMsg;  //!< Flyby diagnostic output message

   private:
    std::unique_ptr<FlybyPointAlgorithm> algorithm = nullptr;
};

#endif  // F32XMERA_FLYBY_POINT_H
