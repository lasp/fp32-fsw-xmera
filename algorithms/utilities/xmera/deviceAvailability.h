#ifndef F32XMERA_XMERA_DEVICE_AVAILABILITY_H
#define F32XMERA_XMERA_DEVICE_AVAILABILITY_H

#include "utilities/fsw/deviceAvailability.h"

#include <fswAlgorithms/fswUtilities/fswDefinitions.h>

// Maps the FSWdeviceAvailability that xmera's message payloads carry onto the availability type the
// algorithms use.
//
// This header is for adapter use (hosted environment) only. It includes an xmera header, which the
// freestanding flight build has no path to, so algorithm and C shim code must reach for
// utilities/fsw/deviceAvailability.h instead.

namespace fsw {

inline DeviceAvailability mapStatus(const ::FSWdeviceAvailability availability) {
    return availability == AVAILABLE ? DeviceAvailability::Available : DeviceAvailability::Unavailable;
}

inline ::FSWdeviceAvailability toXmeraStatus(const DeviceAvailability availability) {
    return availability == DeviceAvailability::Available ? AVAILABLE : UNAVAILABLE;
}

}  // namespace fsw

#endif  // F32XMERA_XMERA_DEVICE_AVAILABILITY_H
