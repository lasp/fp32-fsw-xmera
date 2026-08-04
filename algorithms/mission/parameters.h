#ifndef MISSION_PARAMETERS_H
#define MISSION_PARAMETERS_H

// Mission sizing constants for EMA. This header is the single source of the array
// bounds shared by the algorithms, their C boundary types and the message payloads.
// A Xmera build selects it by pointing XMERA_MISSION_PARAMETERS_DIR at the directory
// holding this "mission" folder, so it defines every constant Xmera expects, including
// ones not used by fp32 algorithms.

#define MAX_NUM_CSS_SENSORS 12
#define MAX_EFF_CNT 12
#define RW_EFF_CNT 4
#define MAX_N_CSS_MEAS 32

#define MAX_KEY_POINTS 5000

#define MAX_SICP_POINTS 5000
#define SICP_POINT_DIM 3
#define MAX_SICP_ITERATIONS 250

#define MAX_NUMBER_REGIONS 3

#endif  // MISSION_PARAMETERS_H
