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
    std::tuple<Eigen::Vector3f, Eigen::Vector3f> readRelativeState();

    // Phase 1: Public config properties — set before reset()
    double controlPeriod = 0.0;       //!< [s] time between updateState() calls; must match the task rate (> 0)
    uint32_t filterReadPeriods = 1U;  //!< [-] control periods between two consecutive filter re-reads (>= 1)
    float toleranceForCollinearity = 0.0F;
    int signOfOrbitNormalFrameVector = 1;
    float maximumRateThreshold = 0.0F;
    float maximumAccelerationThreshold = 0.0F;
    float numberOfSigmas = 0.0F;  //!< [-] number of propagated ground sigmas a re-read may deviate (> 0)
    // Ground-based state that seeds the profile, at the first updateState() after reset(). Required: the zero
    // defaults are rejected, and r and v must be usable and not collinear.
    Eigen::Vector3d groundPosition_BN_N = Eigen::Vector3d::Zero();  //!< [m] ground-based position
    Eigen::Vector3d groundVelocity_BN_N = Eigen::Vector3d::Zero();  //!< [m/s] ground-based velocity
    Eigen::Vector3d groundPositionCovarianceDiag_N =
        Eigen::Vector3d::Zero();  //!< [m^2] diagonal of the ground-based position covariance (> 0)
    Eigen::Vector3d groundVelocityCovarianceDiag_N =
        Eigen::Vector3d::Zero();  //!< [(m/s)^2] diagonal of the ground-based velocity covariance (> 0)

    ReadFunctor<NavTransMsgF32Payload> filterInMsg;               //!< input msg relative position w.r.t. asteroid
    Message<AttRefMsgF32Payload> attRefOutMsg;                    //!< Attitude reference output message
    Message<FlybyDiagnosticMsgF32Payload> flybyDiagnosticOutMsg;  //!< Flyby diagnostic output message

   private:
    std::unique_ptr<FlybyPointAlgorithm> algorithm = nullptr;
};

#endif  // F32XMERA_FLYBY_POINT_H
