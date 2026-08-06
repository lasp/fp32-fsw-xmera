#ifndef F32XMERA_FSW_DEVICE_AVAILABILITY_H
#define F32XMERA_FSW_DEVICE_AVAILABILITY_H

/**
 * @brief Availability state of a device, as carried across the C boundary.
 *
 * The values match xmera's FSWdeviceAvailability, which the message payloads use, but this header
 * deliberately does not include the xmera definition: it is compiled into the freestanding flight
 * build, which has no xmera on the include path. utilities/xmera/deviceAvailability.h maps between
 * the two for adapter code. The enumerators are spelled differently from xmera's AVAILABLE and
 * UNAVAILABLE because a hosted build already has those names at global scope.
 */
typedef enum { DEVICE_AVAILABLE = 0, DEVICE_UNAVAILABLE = 1 } DeviceAvailability_c;

#ifdef __cplusplus

namespace fsw {

/*! @brief Availability state of a device, as used by the algorithms. */
enum class DeviceAvailability {
    Available = DEVICE_AVAILABLE,     //!< the device may be used
    Unavailable = DEVICE_UNAVAILABLE  //!< the device must be excluded
};

inline DeviceAvailability mapStatus(const DeviceAvailability_c availability) {
    return availability == DEVICE_AVAILABLE ? DeviceAvailability::Available : DeviceAvailability::Unavailable;
}

inline DeviceAvailability_c mapStatus(const DeviceAvailability availability) {
    return static_cast<DeviceAvailability_c>(availability);
}

}  // namespace fsw

#endif  // __cplusplus

#endif  // F32XMERA_FSW_DEVICE_AVAILABILITY_H
