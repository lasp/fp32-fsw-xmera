#ifndef AVERAGE_RW_SPEED_DATA_H
#define AVERAGE_RW_SPEED_DATA_H

#include "averageRwSpeedDataAlgorithm.h"
#include "msgPayloadDef/RWSpeedMsgF32Payload.h"
#include <architecture/messaging/messaging.h>

#include <memory>

class AverageRwSpeedData final : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;
    void reconfigure() const;  //!< Re-push the config into the running algorithm; runtime state is untouched
    void reInitialize();       //!< Re-seed the algorithm's runtime state from the configured values

    // Phase 1: public configuration properties -- set before reset().
    double rwSpeedAveragingWindow = 0.0;  //!< [s] RW speed averaging window

    Message<RWSpeedMsgF32Payload> rwSpeedOutMsg;
    ReadFunctor<RWSpeedMsgF32Payload> rwSpeedInMsg;

   private:
    AverageRwSpeedDataConfig toConfig() const;

    uint64_t prevInMsgTime = 0; /*!< [ns] Measurement time of the previous message*/

    std::unique_ptr<AverageRwSpeedDataAlgorithm> algorithm = nullptr;
};
#endif
