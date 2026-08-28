#ifndef F32XMERA_THR_DESAT_DUTY_CYCLE_TYPES_H
#define F32XMERA_THR_DESAT_DUTY_CYCLE_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Maximum number of thrusters handled at the C boundary. Must match kMaxThrusterCount in
    msgPayloadDef/definitions.h (enforced by a static_assert in the C shim). */
#define THR_DESAT_DUTY_CYCLE_MAX_THRUSTER_COUNT 36

/**
 * @brief Bounded-array carrier for the per-thruster force command, used for both the gate's input and its
 *        output. It exists because a C function cannot return a bare array; the C++ algorithm passes the same
 *        data as a std::array. Entries beyond the installed thruster count describe no real thruster and are
 *        gated like the rest, so they stay zero for a zero input.
 */
typedef struct {
    float thrForce[THR_DESAT_DUTY_CYCLE_MAX_THRUSTER_COUNT]; /*!< [N] per-thruster force command */
} ThrDesatDutyCycleForceCmd_c;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif /* F32XMERA_THR_DESAT_DUTY_CYCLE_TYPES_H */
