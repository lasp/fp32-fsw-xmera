#ifndef MIMU_PACKET_GROUP_F32_PAYLOAD_H
#define MIMU_PACKET_GROUP_F32_PAYLOAD_H

#define MAX_MIMU_SAMPLES_PER_PKT 10

#include "AccPktDataMsgF32Payload.h"

/*! @brief A single MIMU packet: a fixed-size group of time-tagged
 *         gyro/accel samples produced by one MIMU at a contiguous burst
 *         rate. The samples within a packet are independent measurements --
 *         each carries its own measTime -- and downstream consumers may
 *         filter them individually. */
typedef struct {
    AccPktDataMsgF32Payload samples[MAX_MIMU_SAMPLES_PER_PKT];  //!< [-] Individual time-tagged MIMU samples
} MimuPacketGroupF32Payload;

#endif
