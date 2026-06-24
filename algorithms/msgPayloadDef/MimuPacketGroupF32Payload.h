#ifndef MIMU_PACKET_GROUP_F32_PAYLOAD_H
#define MIMU_PACKET_GROUP_F32_PAYLOAD_H

#define MAX_MIMU_SAMPLES_PER_PKT 10

#include "AccPktDataMsgF32Payload.h"

#include <stdint.h>

/*! @brief A single MIMU packet: a fixed-size group of gyro/accel samples
 *         produced by one MIMU at a contiguous burst rate. `measTime` is
 *         the timestamp of the first sample in the group; consumers that
 *         care about per-sample timing derive it as
 *         `measTime + s * device_sample_period_ns`. Each
 *         AccPktDataMsgF32Payload retains its own per-sample `measTime`
 *         for legacy consumers but the group-level `measTime` is the
 *         canonical timestamp for this packet. */
typedef struct {
    uint64_t measTime;  //!< [ns] First sample's measurement time
    AccPktDataMsgF32Payload
        samples[MAX_MIMU_SAMPLES_PER_PKT];  //!< [-] Time-tagged MIMU samples (per-sample measTime is legacy)
} MimuPacketGroupF32Payload;

#endif
