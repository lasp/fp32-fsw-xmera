#ifndef F32XMERA_AVERAGE_MIMU_DATA_TYPES_H
#define F32XMERA_AVERAGE_MIMU_DATA_TYPES_H

#include <utilities/fsw/plainCAlgorithmDataTypes.h>

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_MIMU_PKT_C 4
#define MAX_MIMU_SAMPLES_PER_PKT_C 10

/**
 * @brief POD equivalent of one MIMU sample (algorithm-internal `Sample`).
 *
 * Per-sample timestamps are derived from the enclosing packet's `measTime`
 * and the MIMU device's compile-time sample period; the sample itself
 * carries only the gyro/accel measurement.
 */
typedef struct {
    Vector3f_c gyro_P;
    Vector3f_c accel_P;
} Sample_c;

/**
 * @brief POD equivalent of one MIMU packet (algorithm-internal `InputPacket`).
 *
 * `isValid` gates the whole packet; when true, all samples in the packet are
 * assumed real. `measTime` is the timestamp of the first sample.
 */
typedef struct {
    bool isValid;
    uint64_t measTime;
    Sample_c samples[MAX_MIMU_SAMPLES_PER_PKT_C];
} InputPacket_c;

/**
 * @brief POD equivalent of InputPktsData.
 *
 * 4 packets x 10 samples per packet.
 */
typedef struct {
    InputPacket_c packets[MAX_MIMU_PKT_C];
} InputPktsData_c;

/**
 * @brief POD equivalent of OutputAverageAccelAngleVel.
 */
typedef struct {
    Vector3f_c accel_B;
    Vector3f_c gyroOmega_B;
} OutputAverageAccelAngleVel_c;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_AVERAGE_MIMU_DATA_TYPES_H
