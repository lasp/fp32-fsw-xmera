#ifndef F32XIMERA_AVERAGE_MIMU_DATA_TYPES_H
#define F32XIMERA_AVERAGE_MIMU_DATA_TYPES_H

#include "msgPayloadDef/MimuPacketF32Payload.h"

#include <Eigen/Core>

#ifdef __cplusplus
extern "C" {
#endif

/*! @brief One MIMU sample at the algorithm-internal layer: a time-tagged
 *         gyro/accel pair in the platform frame. */
struct Sample {
    std::uint64_t measTime{0U};
    Eigen::Vector3f gyro_P{Eigen::Vector3f::Zero()};
    Eigen::Vector3f accel_P{Eigen::Vector3f::Zero()};
};

/*! @brief Algorithm-internal view of a MimuPacketF32Payload: 4 packets,
 *         each holding up to MAX_MIMU_SAMPLES_PER_PKT time-tagged samples.
 *         The per-packet isValid flag gates the whole packet; within a
 *         fresh packet, a sample with measTime == 0 is treated as
 *         unfilled. */
struct InputPktsData {
    std::array<bool, MAX_MIMU_PKT> isValid{};
    std::array<std::array<Sample, MAX_MIMU_SAMPLES_PER_PKT>, MAX_MIMU_PKT> samples{};
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
