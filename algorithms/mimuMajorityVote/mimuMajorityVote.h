#ifndef MIMU_MAJORITY_VOTE
#define MIMU_MAJORITY_VOTE

#include "architecture/messaging/messaging.h"
#include "mimuMajorityVoteAlgorithm.h"
#include "msgPayloadDef/IMUSensorBodyMsgF32Payload.h"
#include "msgPayloadDef/MimuFaultMsgPayload.h"

#include <Eigen/Core>
#include <array>
#include <cstdint>
#include <memory>

/*! @brief Inertial Measurement Unit (IMU) sensor container class */
class ImuMessage {
   public:
    ReadFunctor<IMUSensorBodyMsgF32Payload> imuSensorBodyInMsg;  //!< imu input message
};

/*!@brief Adapter class to kick off the computation of majority voted imu data. */
class MimuMajorityVote final : public SysModel {
   public:
    MimuMajorityVote() = default;
    ~MimuMajorityVote() override = default;

    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;
    void reConfigure();                       //!< Re-validate the public parameters onto the live algorithm
    void reInitialize();                      //!< Reset the algorithm's fault persistence counters
    void addImuInput(const ImuMessage& imu);  //!< Method to add imus to the computation

    float omegaThreshold{1.0F};          //!< [rad/s] threshold to determine if a MIMU is faulted
    uint32_t faultPersistenceLimit{1U};  //!< [-] consecutive faults needed to trigger faultDetected

    Message<IMUSensorBodyMsgF32Payload> imuSensorBodyOutMsg;
    Message<MimuFaultMsgPayload> mimuFaultMsg;

   private:
    size_t actualNumberOfImus = 0U;
    std::unique_ptr<MimuMajorityVoteAlgorithm> algorithm = nullptr;
    std::array<ImuMessage, kMimuCount> imuMessages;
};

#endif
