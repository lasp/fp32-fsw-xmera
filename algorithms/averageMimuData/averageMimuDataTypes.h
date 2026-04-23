#ifndef F32XIMERA_AVERAGE_MIMU_DATA_TYPES_H
#define F32XIMERA_AVERAGE_MIMU_DATA_TYPES_H

#include <Eigen/Core>

#ifdef __cplusplus
extern "C" {
#endif

constexpr std::size_t MAX_BUF_PKT = 120;

/*! @brief Structure containing the InputPktsData*/
struct InputPktsData {
    std::array<bool, MAX_BUF_PKT> isValid{};
    std::array<std::uint64_t, MAX_BUF_PKT> measTime{};
    std::array<Eigen::Vector3f, MAX_BUF_PKT> gyro_P{};
    std::array<Eigen::Vector3f, MAX_BUF_PKT> accel_P{};
};

/*! @brief Structure containing the OutputAverageAccelAngleVel*/
struct OutputAverageAccelAngleVel {
    Eigen::Vector3f accel_B = Eigen::Vector3f::Zero();
    Eigen::Vector3f gyroOmega_B = Eigen::Vector3f::Zero();
};

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XIMERA_AVERAGE_MIMU_DATA_TYPES_H
