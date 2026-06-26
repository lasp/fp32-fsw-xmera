#ifndef AVERAGE_MIMU_DATA_H
#define AVERAGE_MIMU_DATA_H

#include "averageMimuDataAlgorithm.h"
#include "msgPayloadDef/IMUSensorBodyMsgF32Payload.h"
#include "msgPayloadDef/MimuPacketF32Payload.h"
#include <architecture/messaging/messaging.h>

class AverageMimuData : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;
    void setGyroAveragingWindow(double window);           //!< [s] Setter method for gyroAveragingWindow
    double getGyroAveragingWindow() const;                //!< [s] Getter method for gyroAveragingWindow
    void setAccelAveragingWindow(double window);          //!< [s] Setter method for accelAveragingWindow
    double getAccelAveragingWindow() const;               //!< [s] Getter method for accelAveragingWindow
    void setDcmPltfToBdy(Eigen::Matrix3f const& dcm_BP);  //!< Setter method for dcm from platform to body
    Eigen::Matrix3f getDcmPltfToBdy() const;              //!< Getter method for dcm from platform to body

    Message<IMUSensorBodyMsgF32Payload> imuOutMsg;
    ReadFunctor<MimuPacketF32Payload> mimuPacketInMsg;

   private:
    uint64_t prevInMsgTime = 0;  /*!< [ns] Measurement time of the previous message*/
    uint64_t staleDataCount = 0; /*!< [-] Counter for cases where measurement data was stale*/

    AverageMimuDataAlgorithm algorithm{};
};
#endif
