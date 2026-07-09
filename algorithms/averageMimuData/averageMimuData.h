#ifndef AVERAGE_MIMU_DATA_H
#define AVERAGE_MIMU_DATA_H

#include "averageMimuDataAlgorithm.h"
#include "msgPayloadDef/IMUSensorBodyMsgF32Payload.h"
#include "msgPayloadDef/MimuPacketF32Payload.h"
#include <architecture/messaging/messaging.h>

#include <memory>

class AverageMimuData final : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;
    void reconfigure() const;  //!< Re-push the config into the running algorithm; runtime state is untouched
    void reInitialize();       //!< Re-seed the algorithm's runtime state from the configured values

    // Phase 1: public configuration properties -- set before reset().
    double gyroAveragingWindow = 0.0;                      //!< [s] Gyro averaging window
    double accelAveragingWindow = 0.0;                     //!< [s] Accel averaging window
    Eigen::Matrix3f dcm_BP = Eigen::Matrix3f::Identity();  //!< [-] Transformation from the platform frame to body

    Message<IMUSensorBodyMsgF32Payload> imuOutMsg;
    ReadFunctor<MimuPacketF32Payload> mimuPacketInMsg;

   private:
    AverageMimuDataConfig toConfig() const;

    uint64_t prevInMsgTime = 0;  /*!< [ns] Measurement time of the previous message*/
    uint64_t staleDataCount = 0; /*!< [-] Counter for cases where measurement data was stale*/

    std::unique_ptr<AverageMimuDataAlgorithm> algorithm = nullptr;
};
#endif
