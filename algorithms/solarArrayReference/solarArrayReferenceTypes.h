#ifndef F32XMERA_SOLAR_ARRAY_REFERENCE_TYPES_H
#define F32XMERA_SOLAR_ARRAY_REFERENCE_TYPES_H

#include "utilities/fsw/plainCAlgorithmDataTypes.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*! @brief Solar array reference tracking modes. */
typedef enum TrackingMode { AUTO_TRACK = 0, SPECIFIED_ANGLE = 1 } TrackingMode;

/**
 * @brief Plain-old-data mirror of the C++ SolarArrayReferenceConfig.
 *
 * The C++ side validates each field via SolarArrayReferenceConfig::create and throws on invalid input: driveAxis
 * and surfaceNormal must be finite, (near-)unit and mutually orthogonal; alignmentThreshold must be in [1e-3, pi/2];
 * trackingMode must be a valid enumerator; specifiedArrayAngle and offsetAngle must be in [-pi, pi].
 */
typedef struct {
    Vector3f_c driveAxis;      /*!< [-] solar array drive axis in body frame */
    Vector3f_c surfaceNormal;  /*!< [-] solar array surface normal at zero rotation */
    float alignmentThreshold;  /*!< [rad] alignment threshold angle between sun direction and drive axis */
    TrackingMode trackingMode; /*!< [-] array tracking mode */
    float specifiedArrayAngle; /*!< [rad] specified reference array angle when tracking mode is specified angle */
    float offsetAngle;         /*!< [rad] offset angle added to the determined reference angle */
} SolarArrayReferenceConfig_c;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_SOLAR_ARRAY_REFERENCE_TYPES_H
