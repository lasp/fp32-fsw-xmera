/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#ifndef F32XIMERA_OE_STATE_EPHEM_ALGORITHM_H
#define F32XIMERA_OE_STATE_EPHEM_ALGORITHM_H

#include "utilities/orbitalMotion.hpp"
#include <array>
#include <cstddef>

inline constexpr double kTolerance = 1e-10;
inline constexpr std::size_t kMaxOeCoeff = 20;
inline constexpr std::size_t kMaxOeRecords = 10;

#include "oeStateEphemTypes.h"

/*! @brief Structure that defines the layout of an Ephemeris "record."  This is
           basically the set of coefficients for the ephemeris elements and
           the time factors associated with those coefficients
*/
struct ChebyshevFitArc {
    unsigned int numberChebCoefficients{};  //!< [-] Number chebyshev coefficients loaded into record
    double ephemerisTimeMiddle{};           //!< [s] Ephemeris time (TDB) associated with the mid-point of the curve
    double ephemerisTimeRadius{};           //!< [s] "Radius" of time that curve is valid for (half of total range)
    std::array<double, kMaxOeCoeff>
        radiusPeriapsisCoefficients{};  //!< [-] Set of chebyshev coefficients for radius at periapses
    std::array<double, kMaxOeCoeff> eccentricityCoefficients{};  //!< [-] Set of chebyshev coefficients for eccentricity
    std::array<double, kMaxOeCoeff> inclinationCoefficients{};   //!< [-] Set of chebyshev coefficients for inclination
    std::array<double, kMaxOeCoeff>
        argPeriapsisCoefficients{};  //!< [-] Set of chebyshev coefficients for argument of periapses
    std::array<double, kMaxOeCoeff>
        raanCoefficients{};  //!< [-] Set of chebyshev coefficients for right ascention of the ascending node
    std::array<double, kMaxOeCoeff>
        trueAnomalyCoefficients{};                       //!< [-] Set of chebyshev coefficients for true anomaly angle
    AnomalyType anomalyFlag{AnomalyType::TRUE_ANOMALY};  //!< [-] Flag indicating if the anomaly angle is true or mean
};

/*! @brief Top level structure for the Chebyshev position ephemeris
           fit system.  Allows the user to specify a set of chebyshev
           coefficients and then use the input time to determine where
           a given body is in space
*/
class OEStateEphemAlgorithm {
   public:
    orbitalMotion::CartesianState update(uint64_t callTime);

    void setCentralBodyGravitationalParameter(double gravitationalParameter);
    double getCentralBodyGravitationalParameter() const;
    void setNumberOfArcs(unsigned int arcs);
    unsigned int getNumberOfArcs() const;
    void setEphemerisTimeJ2000(double ephemerisJ2000);
    double getEphemerisTimeJ2000() const;
    void setVehicleTimeOffset(double timeOffset);
    double getVehicleTimeOffset() const;

    void setArcNumberOfCoefficients(unsigned int arcNumber, unsigned int numberOfCoefficients);
    unsigned int getArcNumberOfCoefficients(unsigned int arcNumber) const;

    void setArcMiddleTime(unsigned int arcNumber, double timeMiddle);
    double getArcMiddleTime(unsigned int arcNumber) const;

    void setArcRadiusTime(unsigned int arcNumber, double timeRadius);
    double getArcRadiusTime(unsigned int arcNumber) const;

    void setArcAnomalyFlag(unsigned int arcNumber, const AnomalyType &anomalyFlag);
    AnomalyType getArcAnomalyFlag(unsigned int arcNumber) const;

    void setArcRadiusPeriapsisCoefficients(unsigned int arcNumber,
                                           const std::array<double, kMaxOeCoeff> &radiusPeriapsisCoefficients);
    std::array<double, kMaxOeCoeff> getArcRadiusPeriapsisCoefficients(unsigned int arcNumber) const;
    void setArcEccentricityCoefficients(unsigned int arcNumber,
                                        const std::array<double, kMaxOeCoeff> &eccentricityCoefficients);
    std::array<double, kMaxOeCoeff> getArcEccentricityCoefficients(unsigned int arcNumber) const;
    void setArcInclinationCoefficients(unsigned int arcNumber,
                                       const std::array<double, kMaxOeCoeff> &inclinationCoefficients);
    std::array<double, kMaxOeCoeff> getArcInclinationCoefficients(unsigned int arcNumber) const;
    void setArcArgPeriapsisCoefficients(unsigned int arcNumber,
                                        const std::array<double, kMaxOeCoeff> &argPeriapsisCoefficients);
    std::array<double, kMaxOeCoeff> getArcArgPeriapsisCoefficients(unsigned int arcNumber) const;
    void setArcRaanCoefficients(unsigned int arcNumber, const std::array<double, kMaxOeCoeff> &raanCoefficients);
    std::array<double, kMaxOeCoeff> getArcRaanCoefficients(unsigned int arcNumber) const;
    void setArcTrueAnomalyCoefficients(unsigned int arcNumber,
                                       const std::array<double, kMaxOeCoeff> &trueAnomalyCoefficients);
    std::array<double, kMaxOeCoeff> getArcTrueAnomalyCoefficients(unsigned int arcNumber) const;

   private:
    ChebyshevFitArc findCurrentArc(uint64_t spacecraftClockTime);
    double scaleEphemerisTime(const ChebyshevFitArc &arc) const;
    static orbitalMotion::ClassicalElements evaluateCoefficients(double currentScaledValue,
                                                                 const ChebyshevFitArc &arc);
    bool allParametersNull() const;
    unsigned int numberOfArcs{};
    double currentEphTime{};
    double mu{};  //!< [m3/s^2] Gravitational parameter for center of orbital elements
    std::array<ChebyshevFitArc, kMaxOeRecords> fitCoefficients{};  //!< [-] Array of Chebyshev records for ephemeris
    double ephemerisTime{};
    double vehicleTimeOffset{};
};

#endif
