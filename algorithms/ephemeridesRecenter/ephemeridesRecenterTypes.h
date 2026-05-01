#ifndef F32XIMERA_EPHEMERIDES_RECENTER_TYPES_H
#define F32XIMERA_EPHEMERIDES_RECENTER_TYPES_H

#define MAX_NUM_CHANGE_BODIES 20

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Identifies a celestial body to be re-centered and the SPICE ID of
 *        the central body its input ephemeris is currently expressed about.
 */
typedef struct {
    int bodySpiceId;           /*!< SPICE ID of the body */
    int originalCentralBodyId; /*!< SPICE ID of the body's original central body */
} BodyToRecenter;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XIMERA_EPHEMERIDES_RECENTER_TYPES_H
