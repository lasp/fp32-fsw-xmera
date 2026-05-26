#ifndef F32XMERA_OE_STATE_EPHEM_TYPES_H
#define F32XMERA_OE_STATE_EPHEM_TYPES_H

#define MAX_OE_COEFF 20
#define MAX_OE_RECORDS 10

#ifdef __cplusplus
extern "C" {
#endif

/*! @brief Enum indicating whether the anomaly angle is true anomaly or mean anomaly */
typedef enum AnomalyType { TRUE_ANOMALY = 0, MEAN_ANOMALY = 1 } AnomalyType;

/**
 * @brief POD representation of Chebyshev coefficients array.
 */
typedef struct {
    double data[MAX_OE_COEFF]; /*!< Chebyshev coefficient values */
} OeCoefficients;

/**
 * @brief POD representation of Cartesian state (position and velocity).
 */
typedef struct {
    double position[3]; /*!< [m] Position vector */
    double velocity[3]; /*!< [m/s] Velocity vector */
} CartesianState_c;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_OE_STATE_EPHEM_TYPES_H
