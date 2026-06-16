#ifndef F32XMERA_EPHEMERIDES_RECENTER_TYPES_H
#define F32XMERA_EPHEMERIDES_RECENTER_TYPES_H

#include <stdint.h>

#define MAX_NUM_CHANGE_BODIES 20

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Plain-old-data mirror of the C++ EphemeridesRecenterConfig.
 *
 *  - bodyCount must not exceed MAX_NUM_CHANGE_BODIES
 *  - newCentralBodyId must be present in bodyIds[0 .. bodyCount-1]
 *  - the moon topology must be valid: every moon's parent is in the list and itself orbits the common
 *    center, with at most one moon per parent
 */
typedef struct {
    int newCentralBodyId;                              /*!< SPICE ID of the new central body */
    int previousCentralBodyId;                         /*!< SPICE ID of the previous common central body */
    int bodyIds[MAX_NUM_CHANGE_BODIES];                /*!< SPICE IDs of every configured body */
    int originalCentralBodyIds[MAX_NUM_CHANGE_BODIES]; /*!< original central-body SPICE ID for each body */
    uint32_t bodyCount;                                /*!< number of configured bodies */
} EphemeridesRecenterConfig_c;

/**
 * @brief POD representation of one body's ephemeris payload as used by the
 *        algorithm. The C++ side uses Eigen::Vector3d for r/v vectors and
 *        a bool for the moon flag; both are converted in the shim.
 */
typedef struct {
    int bodySpiceId;           /*!< SPICE ID of the body */
    int originalCentralBodyId; /*!< SPICE ID of original central body */
    int isMoon;                /*!< 1 if this body is a moon of another listed body, else 0 */
    double input_r[3];         /*!< [m] input position */
    double input_v[3];         /*!< [m/s] input velocity */
    double output_r[3];        /*!< [m] output position relative to new central body */
    double output_v[3];        /*!< [m/s] output velocity relative to new central body */
} BodyEphemerisPayload_c;

/**
 * @brief Bounded array of MAX_NUM_CHANGE_BODIES body payloads.
 */
typedef struct {
    BodyEphemerisPayload_c body[MAX_NUM_CHANGE_BODIES];
} BodyEphemerisPayloadArray20_c;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_EPHEMERIDES_RECENTER_TYPES_H
