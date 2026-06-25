#ifndef F32XMERA_CELESTIAL_TWO_BODY_POINT_TYPES_H
#define F32XMERA_CELESTIAL_TWO_BODY_POINT_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Plain-old-data mirror of the C++ CelestialTwoBodyPointConfig fields.
 *
 * Caller fills this struct and passes it to CelestialTwoBodyPointAlgorithm_create or _setConfig.
 * The C++ side validates each field via CelestialTwoBodyPointConfig::create and throws on
 * invalid input.
 *  - celestialBodyAlignmentThreshold must be >= 1e-6
 */
typedef struct {
    float celestialBodyAlignmentThreshold; /*!< [rad] Angle threshold for primary and secondary celestial body alignment
                                              check */
} CelestialTwoBodyPointConfig_c;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_CELESTIAL_TWO_BODY_POINT_TYPES_H
