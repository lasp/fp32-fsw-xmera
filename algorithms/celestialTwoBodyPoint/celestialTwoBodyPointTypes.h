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
 *  - singularityThreshold must be >= 0
 *  - rateThreshold must be >= 0
 *  - secCelBodyIsLinked is unconstrained (interpreted as boolean: zero = not linked,
 *    non-zero = linked)
 */
typedef struct {
    float singularityThreshold; /*!< [rad] Angle threshold below which the constraint axis is fixed */
    float rateThreshold;        /*!< [rad/s] Rate threshold above which the constraint axis is fixed */
    int secCelBodyIsLinked;     /*!< [-] Whether the secondary celestial body input is available */
} CelestialTwoBodyPointConfig_c;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_CELESTIAL_TWO_BODY_POINT_TYPES_H
