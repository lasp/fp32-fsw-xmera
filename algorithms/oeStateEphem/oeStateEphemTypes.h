#ifndef F32XMERA_OE_STATE_EPHEM_TYPES_H
#define F32XMERA_OE_STATE_EPHEM_TYPES_H

#include <stdint.h>

#define MAX_OE_COEFF 20
#define MAX_OE_RECORDS 10

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @brief Enum indicating whether the anomaly angle is true anomaly or mean anomaly.
 *
 * The underlying type is fixed at uint8_t to match the Ada side's 8-bit Anomaly_Flag.
 * A plain C enum is int-width, which would read four bytes where Ada wrote one.
 */
typedef enum AnomalyType : uint8_t { TRUE_ANOMALY = 0, MEAN_ANOMALY = 1 } AnomalyType;

/**
 * @brief POD representation of Cartesian state (position and velocity).
 */
typedef struct {
    double position[3]; /*!< [m] Position vector */
    double velocity[3]; /*!< [m/s] Velocity vector */
} CartesianState_c;

/**
 * @brief POD representation of a single Chebyshev fit arc for the C/Ada boundary.
 *
 * Field order mirrors the Adamant Oe_Arc packed record (oe_arc.record.yaml), which is
 * the uplinked parameter-table format and therefore the layout that wins. ChebyshevFitArc
 * is kept in the same order so all three declarations read alike.
 *
 * The arc array crosses by reference, so the C++ side reads it at fixed offsets:
 * reordering a field or changing anomalyFlag's width here silently misreads data rather
 * than failing to compile. The Ada binding's
 * Oe_Arc.C.U_C'Object_Size = getFitArcSizeBits() assert does not catch either case --
 * the seven bytes of padding after anomalyFlag absorb any width up to 64 bits, and a
 * size-preserving reorder leaves the total unchanged. That assert only catches a field
 * added or removed, or a double narrowed. The behavioural component tests are what
 * actually guard field order and flag width.
 */
typedef struct {
    unsigned int numberChebCoefficients;              /*!< [-] number of Chebyshev coefficients in the arc */
    double ephemerisTimeMiddle;                       /*!< [s] ephemeris time at the arc mid-point */
    double ephemerisTimeRadius;                       /*!< [s] half-width of the arc's valid time range */
    AnomalyType anomalyFlag;                          /*!< [-] 0 = TRUE_ANOMALY, 1 = MEAN_ANOMALY */
    double radiusPeriapsisCoefficients[MAX_OE_COEFF]; /*!< [-] radius-of-periapsis coefficients */
    double eccentricityCoefficients[MAX_OE_COEFF];    /*!< [-] eccentricity coefficients */
    double inclinationCoefficients[MAX_OE_COEFF];     /*!< [-] inclination coefficients */
    double argPeriapsisCoefficients[MAX_OE_COEFF];    /*!< [-] argument-of-periapsis coefficients */
    double raanCoefficients[MAX_OE_COEFF];            /*!< [-] right-ascension-of-ascending-node coefficients */
    double trueAnomalyCoefficients[MAX_OE_COEFF];     /*!< [-] anomaly-angle coefficients */
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
