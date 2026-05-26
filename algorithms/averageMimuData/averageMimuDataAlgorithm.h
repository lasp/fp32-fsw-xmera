#ifndef AVERAGE_MIMU_DATA_ALGORITHM_H
#define AVERAGE_MIMU_DATA_ALGORITHM_H

#include "averageMimuDataTypes.h"
#include "msgPayloadDef/AccDataMsgF32Payload.h"

#include <Eigen/Core>
#include <array>
#include <cstdint>

constexpr std::size_t MAX_BUF_PKT = 120;

/*! @brief Structure containing the InputPktsData*/
struct InputPktsData {
    std::array<std::uint64_t, MAX_BUF_PKT> measTime{};
    std::array<Eigen::Vector3f, MAX_BUF_PKT> gyro_P{};
    std::array<Eigen::Vector3f, MAX_BUF_PKT> accel_P{};
};

/*! @brief Structure containing the OutputAverageAccelAngleVel*/
struct OutputAverageAccelAngleVel {
    Eigen::Vector3f accel_B = Eigen::Vector3f::Zero();
    Eigen::Vector3f gyroOmega_B = Eigen::Vector3f::Zero();
};

class AverageMimuDataAlgorithm {
   public:
    void setAveragingWindow(float window);                  //!< [s] Setter method for windowSec
    float getAveragingWindow() const;                       //!< [s] Getter method for windowSec
    void setDcmPltfToBdy(Eigen::Matrix3f const& dcm_BPIn);  //!< Setter method for dcm from platform to body
    Eigen::Matrix3f getDcmPltfToBdy() const;                //!< Getter method for dcm from platform to body
    OutputAverageAccelAngleVel update(InputPktsData const& localPkts) const;

   private:
    float averagingWindow{0.0F};                           //!< [s] Allowable time difference from "latest"
    Eigen::Matrix3f dcm_BP = Eigen::Matrix3f::Identity();  //!< [-] Transformation from the platform frame to body
};

#endif
