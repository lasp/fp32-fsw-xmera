#ifndef F32XMERA_THRUST_MAPPING_TYPES_H
#define F32XMERA_THRUST_MAPPING_TYPES_H

#include "msgPayloadDef/definitions.h"
#include "utilities/fsw/plainCAlgorithmDataTypes.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Per-axis controllability assertions, torque xyz then force xyz, all in body frame B.
 *
 * A true entry asserts that axis must lie in the column space of the control mapping matrix DG;
 * it is checked against the SVD when the mapping is computed.
 */
typedef struct {
    bool torqueX;
    bool torqueY;
    bool torqueZ;
    bool forceX;
    bool forceY;
    bool forceZ;
} ForceTorqueControlAxes_c;

/**
 * @brief Plain-old-data mirror of the C++ Eigen::Vector<float, kMaxThrusterCount> update output.
 *
 * Entries 0..numThrusters-1 carry the non-negative per-thruster force commands; trailing slots and
 * unavailable thrusters are exactly zero.
 */
typedef struct {
    float thrForce[MAX_EFF_CNT];
} ThrForceArray_c;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_THRUST_MAPPING_TYPES_H
