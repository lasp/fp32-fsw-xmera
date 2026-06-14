#ifndef F32XMERA_FLYBY_POINT_H
#define F32XMERA_FLYBY_POINT_H

#include "flybyPointAlgorithm.h"
#include "msgPayloadDef/AttRefMsgF32Payload.h"
#include "msgPayloadDef/FlybyDiagnosticMsgF32Payload.h"
#include "msgPayloadDef/NavTransMsgF32Payload.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <Eigen/Dense>

/*! @brief A class to perform flyby pointing */
class FlybyPoint : public SysModel {
   public:
    FlybyPoint();
    void reset(uint64_t currentSimNanos) override;
    void updateState(uint64_t currentSimNanos) override;
    std::tuple<Eigen::Vector3d, Eigen::Vector3d> readRelativeState();
    double getTimeBetweenFilterData() const;
    void setTimeBetweenFilterData(double timeBetweenFilterData);
    float getToleranceForCollinearity() const;
    void setToleranceForCollinearity(float toleranceForCollinearity);
    int getSignOfOrbitNormalFrameVector() const;
    void setSignOfOrbitNormalFrameVector(int signOfOrbitNormalFrameVector);
    float getMaximumAccelerationThreshold() const;
    void setMaximumAccelerationThreshold(float maxAccelerationThreshold);
    float getMaximumRateThreshold() const;
    void setMaximumRateThreshold(float maxRateThreshold);
    float getPositionKnowledgeSigma() const;
    void setPositionKnowledgeSigma(float positionKnowledgeStd);

    ReadFunctor<NavTransMsgF32Payload> filterInMsg;               //!< input msg relative position w.r.t. asteroid
    Message<AttRefMsgF32Payload> attRefOutMsg;                    //!< Attitude reference output message
    Message<FlybyDiagnosticMsgF32Payload> flybyDiagnosticOutMsg;  //!< Flyby diagnostic output message

   private:
    FlybyPointAlgorithm algorithm;
};

#endif  // F32XMERA_FLYBY_POINT_H
