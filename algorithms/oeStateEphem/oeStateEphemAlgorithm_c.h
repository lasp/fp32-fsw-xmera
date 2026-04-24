/* MIT License
 *
 Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#ifndef F32XMERA_OE_STATE_EPHEM_ALGORITHM_C_H
#define F32XMERA_OE_STATE_EPHEM_ALGORITHM_C_H

#include "oeStateEphemTypes.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ OEStateEphemAlgorithm instance.
 */
typedef struct OEStateEphemAlgorithm OEStateEphemAlgorithm;

/**
 * @brief Construct a new OEStateEphemAlgorithm instance.
 * @return Pointer to a new OEStateEphemAlgorithm (must be destroyed).
 */
OEStateEphemAlgorithm* OEStateEphemAlgorithm_create(void);

/**
 * @brief Destroy a previously created OEStateEphemAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void OEStateEphemAlgorithm_destroy(OEStateEphemAlgorithm* self);

/**
 * @brief Run the update step to compute Cartesian state from ephemeris.
 * @param self     Pointer to the instance.
 * @param callTime Clock time in nanoseconds.
 * @return CartesianState_c  The computed position and velocity vectors.
 */
CartesianState_c OEStateEphemAlgorithm_update(OEStateEphemAlgorithm* self, uint64_t callTime);

/**
 * @brief Set the gravitational parameter of the central body.
 * @param self                    Pointer to the instance.
 * @param gravitationalParameter  Gravitational parameter (m^3/s^2).
 */
void OEStateEphemAlgorithm_setCentralBodyGravitationalParameter(OEStateEphemAlgorithm* self,
                                                                double gravitationalParameter);

/**
 * @brief Get the gravitational parameter of the central body.
 * @param self Pointer to the instance.
 * @return double  Gravitational parameter (m^3/s^2).
 */
double OEStateEphemAlgorithm_getCentralBodyGravitationalParameter(const OEStateEphemAlgorithm* self);

/**
 * @brief Set the number of orbital element coefficient arcs.
 * @param self Pointer to the instance.
 * @param arcs Number of arcs.
 */
void OEStateEphemAlgorithm_setNumberOfArcs(OEStateEphemAlgorithm* self, unsigned int arcs);

/**
 * @brief Get the number of orbital element coefficient arcs.
 * @param self Pointer to the instance.
 * @return unsigned int  Number of arcs.
 */
unsigned int OEStateEphemAlgorithm_getNumberOfArcs(const OEStateEphemAlgorithm* self);

/**
 * @brief Set the ephemeris time offset referenced to J2000.
 * @param self           Pointer to the instance.
 * @param ephemerisJ2000 Ephemeris time offset (seconds).
 */
void OEStateEphemAlgorithm_setEphemerisTimeJ2000(OEStateEphemAlgorithm* self, double ephemerisJ2000);

/**
 * @brief Get the ephemeris time offset referenced to J2000.
 * @param self Pointer to the instance.
 * @return double  Ephemeris time offset (seconds).
 */
double OEStateEphemAlgorithm_getEphemerisTimeJ2000(const OEStateEphemAlgorithm* self);

/**
 * @brief Set the vehicle time offset.
 * @param self       Pointer to the instance.
 * @param timeOffset Vehicle time offset (seconds).
 */
void OEStateEphemAlgorithm_setVehicleTimeOffset(OEStateEphemAlgorithm* self, double timeOffset);

/**
 * @brief Get the vehicle time offset.
 * @param self Pointer to the instance.
 * @return double  Vehicle time offset (seconds).
 */
double OEStateEphemAlgorithm_getVehicleTimeOffset(const OEStateEphemAlgorithm* self);

/**
 * @brief Set the number of Chebyshev coefficients for a specified arc.
 * @param self                  Pointer to the instance.
 * @param arcNumber             Index of the arc.
 * @param numberOfCoefficients  Number of coefficients.
 */
void OEStateEphemAlgorithm_setArcNumberOfCoefficients(OEStateEphemAlgorithm* self,
                                                      unsigned int arcNumber,
                                                      unsigned int numberOfCoefficients);

/**
 * @brief Get the number of Chebyshev coefficients for a specified arc.
 * @param self      Pointer to the instance.
 * @param arcNumber Index of the arc.
 * @return unsigned int  Number of coefficients.
 */
unsigned int OEStateEphemAlgorithm_getArcNumberOfCoefficients(const OEStateEphemAlgorithm* self,
                                                              unsigned int arcNumber);

/**
 * @brief Set the middle time for a specified arc.
 * @param self       Pointer to the instance.
 * @param arcNumber  Index of the arc.
 * @param timeMiddle Ephemeris time at arc midpoint (seconds).
 */
void OEStateEphemAlgorithm_setArcMiddleTime(OEStateEphemAlgorithm* self, unsigned int arcNumber, double timeMiddle);

/**
 * @brief Get the middle time for a specified arc.
 * @param self      Pointer to the instance.
 * @param arcNumber Index of the arc.
 * @return double  Ephemeris time at arc midpoint (seconds).
 */
double OEStateEphemAlgorithm_getArcMiddleTime(const OEStateEphemAlgorithm* self, unsigned int arcNumber);

/**
 * @brief Set the time radius for a specified arc.
 * @param self       Pointer to the instance.
 * @param arcNumber  Index of the arc.
 * @param timeRadius Time radius (seconds).
 */
void OEStateEphemAlgorithm_setArcRadiusTime(OEStateEphemAlgorithm* self, unsigned int arcNumber, double timeRadius);

/**
 * @brief Get the time radius for a specified arc.
 * @param self      Pointer to the instance.
 * @param arcNumber Index of the arc.
 * @return double  Time radius (seconds).
 */
double OEStateEphemAlgorithm_getArcRadiusTime(const OEStateEphemAlgorithm* self, unsigned int arcNumber);

/**
 * @brief Set the anomaly flag for a specified arc.
 * @param self        Pointer to the instance.
 * @param arcNumber   Index of the arc.
 * @param anomalyFlag Anomaly type (TRUE_ANOMALY or MEAN_ANOMALY).
 */
void OEStateEphemAlgorithm_setArcAnomalyFlag(OEStateEphemAlgorithm* self,
                                             unsigned int arcNumber,
                                             AnomalyType anomalyFlag);

/**
 * @brief Get the anomaly flag for a specified arc.
 * @param self      Pointer to the instance.
 * @param arcNumber Index of the arc.
 * @return AnomalyType  The anomaly type.
 */
AnomalyType OEStateEphemAlgorithm_getArcAnomalyFlag(const OEStateEphemAlgorithm* self, unsigned int arcNumber);

/**
 * @brief Set the radius periapsis Chebyshev coefficients for a specified arc.
 * @param self         Pointer to the instance.
 * @param arcNumber    Index of the arc.
 * @param coefficients Pointer to the coefficients.
 */
void OEStateEphemAlgorithm_setArcRadiusPeriapsisCoefficients(OEStateEphemAlgorithm* self,
                                                             unsigned int arcNumber,
                                                             const OeCoefficients* coefficients);

/**
 * @brief Get the radius periapsis Chebyshev coefficients for a specified arc.
 * @param self      Pointer to the instance.
 * @param arcNumber Index of the arc.
 * @return OeCoefficients  The coefficients.
 */
OeCoefficients OEStateEphemAlgorithm_getArcRadiusPeriapsisCoefficients(const OEStateEphemAlgorithm* self,
                                                                       unsigned int arcNumber);

/**
 * @brief Set the eccentricity Chebyshev coefficients for a specified arc.
 * @param self         Pointer to the instance.
 * @param arcNumber    Index of the arc.
 * @param coefficients Pointer to the coefficients.
 */
void OEStateEphemAlgorithm_setArcEccentricityCoefficients(OEStateEphemAlgorithm* self,
                                                          unsigned int arcNumber,
                                                          const OeCoefficients* coefficients);

/**
 * @brief Get the eccentricity Chebyshev coefficients for a specified arc.
 * @param self      Pointer to the instance.
 * @param arcNumber Index of the arc.
 * @return OeCoefficients  The coefficients.
 */
OeCoefficients OEStateEphemAlgorithm_getArcEccentricityCoefficients(const OEStateEphemAlgorithm* self,
                                                                    unsigned int arcNumber);

/**
 * @brief Set the inclination Chebyshev coefficients for a specified arc.
 * @param self         Pointer to the instance.
 * @param arcNumber    Index of the arc.
 * @param coefficients Pointer to the coefficients.
 */
void OEStateEphemAlgorithm_setArcInclinationCoefficients(OEStateEphemAlgorithm* self,
                                                         unsigned int arcNumber,
                                                         const OeCoefficients* coefficients);

/**
 * @brief Get the inclination Chebyshev coefficients for a specified arc.
 * @param self      Pointer to the instance.
 * @param arcNumber Index of the arc.
 * @return OeCoefficients  The coefficients.
 */
OeCoefficients OEStateEphemAlgorithm_getArcInclinationCoefficients(const OEStateEphemAlgorithm* self,
                                                                   unsigned int arcNumber);

/**
 * @brief Set the argument of periapsis Chebyshev coefficients for a specified arc.
 * @param self         Pointer to the instance.
 * @param arcNumber    Index of the arc.
 * @param coefficients Pointer to the coefficients.
 */
void OEStateEphemAlgorithm_setArcArgPeriapsisCoefficients(OEStateEphemAlgorithm* self,
                                                          unsigned int arcNumber,
                                                          const OeCoefficients* coefficients);

/**
 * @brief Get the argument of periapsis Chebyshev coefficients for a specified arc.
 * @param self      Pointer to the instance.
 * @param arcNumber Index of the arc.
 * @return OeCoefficients  The coefficients.
 */
OeCoefficients OEStateEphemAlgorithm_getArcArgPeriapsisCoefficients(const OEStateEphemAlgorithm* self,
                                                                    unsigned int arcNumber);

/**
 * @brief Set the RAAN Chebyshev coefficients for a specified arc.
 * @param self         Pointer to the instance.
 * @param arcNumber    Index of the arc.
 * @param coefficients Pointer to the coefficients.
 */
void OEStateEphemAlgorithm_setArcRaanCoefficients(OEStateEphemAlgorithm* self,
                                                  unsigned int arcNumber,
                                                  const OeCoefficients* coefficients);

/**
 * @brief Get the RAAN Chebyshev coefficients for a specified arc.
 * @param self      Pointer to the instance.
 * @param arcNumber Index of the arc.
 * @return OeCoefficients  The coefficients.
 */
OeCoefficients OEStateEphemAlgorithm_getArcRaanCoefficients(const OEStateEphemAlgorithm* self, unsigned int arcNumber);

/**
 * @brief Set the true anomaly Chebyshev coefficients for a specified arc.
 * @param self         Pointer to the instance.
 * @param arcNumber    Index of the arc.
 * @param coefficients Pointer to the coefficients.
 */
void OEStateEphemAlgorithm_setArcTrueAnomalyCoefficients(OEStateEphemAlgorithm* self,
                                                         unsigned int arcNumber,
                                                         const OeCoefficients* coefficients);

/**
 * @brief Get the true anomaly Chebyshev coefficients for a specified arc.
 * @param self      Pointer to the instance.
 * @param arcNumber Index of the arc.
 * @return OeCoefficients  The coefficients.
 */
OeCoefficients OEStateEphemAlgorithm_getArcTrueAnomalyCoefficients(const OEStateEphemAlgorithm* self,
                                                                   unsigned int arcNumber);

/**
 * @brief Get the MAX_OE_COEFF constant for Ada validation.
 * @return The value of MAX_OE_COEFF.
 */
uint32_t OEStateEphemAlgorithm_getMaxOeCoeff(void);

/**
 * @brief Get the MAX_OE_RECORDS constant for Ada validation.
 * @return The value of MAX_OE_RECORDS.
 */
uint32_t OEStateEphemAlgorithm_getMaxOeRecords(void);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_OE_STATE_EPHEM_ALGORITHM_C_H
