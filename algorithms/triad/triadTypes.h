#ifndef F32XMERA_TRIAD_TYPES_H
#define F32XMERA_TRIAD_TYPES_H

#include "utilities/plainCAlgorithmDataTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Plain-old-data mirror of the C++ TriadConfig fields.
 *
 * Caller fills this struct and passes it to TriadAlgorithm_create or _setConfig. The C++ side
 * validates and normalizes each field via TriadConfig::create and throws on invalid input.
 *  - sadaHat_B      [-] solar array drive axis, unit vector in body-frame components
 *  - thrustReqHat_N [-] requested thrust direction, unit vector in inertial-frame components
 *  - signOfZHat_N   [-] sign selecting the inertial z-axis direction used as the fallback
 *                       constraint axis when the sun and thrust reference are aligned; nonzero
 */
typedef struct {
    Vector3f_c sadaHat_B;
    Vector3f_c thrustReqHat_N;
    float signOfZHat_N;
} TriadConfig_c;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_TRIAD_TYPES_H
