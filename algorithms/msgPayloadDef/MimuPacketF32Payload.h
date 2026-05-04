#ifndef MIMU_PACKET_F32_PAYLOAD_H
#define MIMU_PACKET_F32_PAYLOAD_H

#define MAX_MIMU_PKT 4

#include "MimuPacketGroupF32Payload.h"

#include <stdbool.h>

/*! @brief A small ring of MIMU packets from a single source, with a
 *         per-packet validity flag. Each slot carries a
 *         MimuPacketGroupF32Payload (a group of MAX_MIMU_SAMPLES_PER_PKT
 *         time-tagged samples). `isValid[i]` is false for slots that have
 *         not yet been filled (e.g. during ring-buffer warm-up); when
 *         false, every sample in that slot must be ignored. */
typedef struct {
    MimuPacketGroupF32Payload packets[MAX_MIMU_PKT];  //!< [-] MIMU packets from a single source
    bool isValid[MAX_MIMU_PKT];                       //!< [-] True when the corresponding packet holds fresh samples
} MimuPacketF32Payload;

#endif
