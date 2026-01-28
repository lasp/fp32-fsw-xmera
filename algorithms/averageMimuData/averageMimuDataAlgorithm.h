#ifndef AVERAGE_MIMU_DATA_ALGORITHM_H
#define AVERAGE_MIMU_DATA_ALGORITHM_H

#include "msgPayloadDef/AccDataMsgF32Payload.h"
#include "msgPayloadDef/IMUSensorBodyMsgF32Payload.h"

#include <Eigen/Core>

class AverageMimuDataAlgorithm {
   public:
    void setTimeDelta(float timeDeltaIn);                   //!< Setter method for timeDelta
    float getTimeDelta() const;                             //!< Getter method for timeDelta
    void setDcmPltfToBdy(Eigen::Matrix3f const& dcm_BPIn);  //!< Setter method for dcm from platform to body
    Eigen::Matrix3f getDcmPltfToBdy() const;                //!< Getter method for dcm from platform to body
    IMUSensorBodyMsgF32Payload update(AccDataMsgF32Payload const& localPkts) const;

   private:
    float timeDelta{0.0F};                                 //!< [s] Allowable time difference from "latest"
    Eigen::Matrix3f dcm_BP = Eigen::Matrix3f::Identity();  //!< [-] Transformation from the platform frame to body
};

#endif
