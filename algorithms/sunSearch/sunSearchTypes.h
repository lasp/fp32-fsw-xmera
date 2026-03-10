/* MIT License
 *
 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#ifndef F32XIMERA_SUN_SEARCH_TYPES_H
#define F32XIMERA_SUN_SEARCH_TYPES_H

#define NUM_SLEWS 3

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Principle vehicle inertia terms (about point B in frame B).
 */
typedef struct {
    float IxxPntB_B; /*!< principle vehicle inertia term (0, 0) */
    float IyyPntB_B; /*!< principle vehicle inertia term (1, 1) */
    float IzzPntB_B; /*!< principle vehicle inertia term (2, 2) */
} PrincipleInertias;

/**
 * @brief Properties defining a single slew maneuver.
 */
typedef struct {
    float slewTime;      /*!< [s] total time for the three-axes maneuver */
    float slewAngle;     /*!< [rad] total angle sweep around one axis */
    float slewMaxRate;   /*!< [rad/s] maximum spacecraft body rate norm */
    float slewMaxTorque; /*!< [Nm] maximum torque for slew */
    int slewRotAxis;     /*!< [-] axes about which to perform the Sun search */
} SlewProperties;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XIMERA_SUN_SEARCH_TYPES_H
