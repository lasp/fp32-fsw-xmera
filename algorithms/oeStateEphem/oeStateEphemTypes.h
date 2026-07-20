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
 * @brief POD representation of Cartesian state (position and velocity).
 */
typedef struct {
    double position[3]; /*!< [m] Position vector */
    double velocity[3]; /*!< [m/s] Velocity vector */
} CartesianState_c;

/**
 * @brief POD representation of a single Chebyshev fit arc for the C/Ada boundary.
 */
typedef struct {
    unsigned int numberChebCoefficients;              /*!< [-] number of Chebyshev coefficients in the arc */
    double ephemerisTimeMiddle;                       /*!< [s] ephemeris time at the arc mid-point */
    double ephemerisTimeRadius;                       /*!< [s] half-width of the arc's valid time range */
    double radiusPeriapsisCoefficients[MAX_OE_COEFF]; /*!< [-] radius-of-periapsis coefficients */
    double eccentricityCoefficients[MAX_OE_COEFF];    /*!< [-] eccentricity coefficients */
    double inclinationCoefficients[MAX_OE_COEFF];     /*!< [-] inclination coefficients */
    double argPeriapsisCoefficients[MAX_OE_COEFF];    /*!< [-] argument-of-periapsis coefficients */
    double raanCoefficients[MAX_OE_COEFF];            /*!< [-] right-ascension-of-ascending-node coefficients */
    double trueAnomalyCoefficients[MAX_OE_COEFF];     /*!< [-] anomaly-angle coefficients */
    AnomalyType anomalyFlag;                          /*!< [-] true vs mean anomaly flag */
} ChebyshevFitArc_c;

/**
 * @brief POD representation of the full OE state ephemeris configuration for the C/Ada boundary.
 */
typedef struct {
    double centralBodyGravitationalParameter;          /*!< [m^3/s^2] central-body gravitational parameter */
    unsigned int numberOfArcs;                         /*!< [-] number of populated arcs */
    double ephemerisTimeJ2000;                         /*!< [s] ephemeris time offset referenced to J2000 */
    double vehicleTimeOffset;                          /*!< [s] vehicle clock time offset */
    ChebyshevFitArc_c fitCoefficients[MAX_OE_RECORDS]; /*!< [-] table of Chebyshev fit arcs */
} OEStateEphemConfig_c;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_OE_STATE_EPHEM_TYPES_H
