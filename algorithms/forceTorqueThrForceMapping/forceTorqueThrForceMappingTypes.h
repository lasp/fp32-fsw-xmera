#ifndef F32XMERA_FORCE_TORQUE_THR_FORCE_MAPPING_TYPES_H
#define F32XMERA_FORCE_TORQUE_THR_FORCE_MAPPING_TYPES_H

#include "msgPayloadDef/definitions.h"
#include "utilities/fsw/plainCAlgorithmDataTypes.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Plain-old-data mirror of the C++ ThrusterConfiguration fields.
 *  - r_TB_B    [m] location of the thruster in the spacecraft body frame
 *  - tHat_B    [-] unit vector of the thrust direction (must be ~unit within 1e-3 of 1.0)
 */
typedef struct {
    Vector3f_c r_TB_B;
    Vector3f_c tHat_B;
} ThrusterConfiguration_c;

/**
 * @brief Plain-old-data mirror of the C++ ThrusterArrayConfiguration fields.
 *  - numThrusters must be in [1, MAX_EFF_CNT]
 *  - thrusters[i] for i < numThrusters carries each thruster's geometry; trailing slots are ignored
 */
typedef struct {
    uint32_t numThrusters;
    ThrusterConfiguration_c thrusters[MAX_EFF_CNT];
} ThrusterArrayConfiguration_c;

/**
 * @brief Plain-old-data mirror of the C++ ForceTorqueThrForceMappingConfig.
 *
 * Caller fills this struct and passes it to ForceTorqueThrForceMappingAlgorithm_create or
 * _setConfig. The C++ side validates each field via ForceTorqueThrForceMappingConfig::create and
 * throws on invalid input.
 *  - thrusters:          count + per-thruster geometry, each direction ~unit within 1e-3
 *  - centerOfMass_B:     [m] center of mass in body frame, must be finite
 *  - desiredControlAxes: per-axis controllability assertions (torque xyz then force xyz, all in
 *                        body frame B). A non-zero entry asserts that axis must lie in the column
 *                        space of DG; checked against the SVD when the mapping is computed.
 */
typedef struct {
    ThrusterArrayConfiguration_c thrusters;
    Vector3f_c centerOfMass_B;
    uint8_t desiredControlAxes[6];
} ForceTorqueThrForceMappingConfig_c;

/**
 * @brief Plain-old-data mirror of the C++ Eigen::Vector<float, MAX_EFF_CNT> update output.
 *
 * Entries 0..numThrusters-1 (as configured) carry the non-negative, min-shifted per-thruster force
 * commands; trailing slots are exactly zero.
 */
typedef struct {
    float thrForce[MAX_EFF_CNT];
} ThrForceArray_c;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_FORCE_TORQUE_THR_FORCE_MAPPING_TYPES_H
