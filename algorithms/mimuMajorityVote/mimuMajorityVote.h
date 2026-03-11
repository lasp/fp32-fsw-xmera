#ifndef MIMU_MAJORITY_VOTE
#define MIMU_MAJORITY_VOTE

#include <Eigen/Core>
#include <cstdint>

#include "architecture/messaging/messaging.h"
#include "mimuMajorityVoteAlgorithm.h"
#include "msgPayloadDef/IMUSensorBodyMsgF32Payload.h"
#include "msgPayloadDef/MimuFaultMsgPayload.h"

/*! @brief Inertial Measurement Unit (IMU) sensor container class */
class ImuMessage {
   public:
    ReadFunctor<IMUSensorBodyMsgF32Payload> imuSensorBodyInMsg;  //!< imu input message
};

/*!@brief Adapter class to kick off the computation of majority voted imu data. */
class MimuMajorityVote : public SysModel {
   public:
    void reset(uint64_t callTime) final;
    void updateState(uint64_t callTime) final;
    void addImuInput(const ImuMessage& imu);       //!< Method to add imus to the computation
    void setOmegaThreshold(float omegaThreshold);  //!< Setter method for omegaThreshold
    float getOmegaThreshold() const;               //!< Getter method for omegaThreshold
    void setNumberOfImus(size_t numberOfImusIn);   //!< Setter method for expected number of IMUs
    size_t getNumberOfImus() const;                //!< Getter method for expected number of IMUs

    Message<IMUSensorBodyMsgF32Payload> imuSensorBodyOutMsg;
    Message<MimuFaultMsgPayload> mimuFaultMsg;

   private:
    size_t actualNumberOfImus = 0U;
    MimuMajorityVoteAlgorithm algorithm{};
    std::array<ImuMessage, MAX_IMU_VEH_COUNT> imuMessages;
};

#endif
