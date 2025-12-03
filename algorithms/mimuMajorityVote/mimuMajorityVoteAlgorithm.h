#ifndef MIMU_MAJORITY_VOTE_ALGORITHM
#define MIMU_MAJORITY_VOTE_ALGORITHM

#include "msgPayloadDef/IMUSensorBodyMsgF32Payload.h"
#include "msgPayloadDef/MimuFaultMsgPayload.h"

#include <Eigen/Core>
#include <cstdint>

constexpr uint32_t MAX_IMU_VEH_COUNT = 4U;

struct MimuMajorityVoteOutput {
    IMUSensorBodyMsgF32Payload imuSensorBodyMsgF32Payload;
    MimuFaultMsgPayload mimuFaultMsgPayload;
};

/*!@brief Module to compute the majority vote of the mimus. */
class MimuMajorityVoteAlgorithm {
   public:
    MimuMajorityVoteOutput update(std::array<IMUSensorBodyMsgF32Payload, MAX_IMU_VEH_COUNT> imuPayloads,
                                  size_t numberOfImus);
    void setOmegaThreshold(float omegaThresholdIn);  //!< Setter method for omegaThreshold
    float getOmegaThreshold() const;                 //!< Getter method for omegaThreshold

   private:
    bool faultDetected = false;
    float omegaThreshold = 1.0F;  // The threshold in which we will determine one of the mimus is faulted (rad/s)
    std::array<float, MAX_IMU_VEH_COUNT - 1U> omegaDifferencesMag = {};
};

#endif
