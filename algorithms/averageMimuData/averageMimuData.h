#ifndef AVERAGE_MIMU_DATA_H
#define AVERAGE_MIMU_DATA_H

#include "architecture/messaging/messaging.h"
#include "averageMimuDataAlgorithm.h"
#include "msgPayloadDef/IMUSensorBodyMsgF32Payload.h"

#include <cstdint>

class AverageMimuData : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;
    void setAveragingWindow(float window);                //!< [s] Setter method for averagingWindow
    float getAveragingWindow() const;                     //!< [s] Getter method for averagingWindow
    void setDcmPltfToBdy(Eigen::Matrix3f const& dcm_BP);  //!< Setter method for dcm from platform to body
    Eigen::Matrix3f getDcmPltfToBdy() const;              //!< Getter method for dcm from platform to body

    Message<IMUSensorBodyMsgF32Payload> imuOutMsg;
    ReadFunctor<AccDataMsgF32Payload> accDataInMsg;

   private:
    uint64_t prevInMsgTime = 0;  /*!< [ns] Measurement time of the previous message*/
    uint64_t staleDataCount = 0; /*!< [-] Counter for cases where measurement data was stale*/

    AverageMimuDataAlgorithm algorithm{};
};
#endif
