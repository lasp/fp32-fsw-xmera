/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#include "oeStateEphemAlgorithm.h"
#include "../freestandingInvalidArgument.h"
#include "utilities/ephemerisUtilities.h"
#include <ranges>

/*! This method finds the Chebyshev fit arc that is closest in time to the current ephemeris time.
    It computes the current ephemeris time from the call time, ephemeris time offset, and vehicle time,
    then searches through all available arcs to find the one with the smallest time difference.
    @return ChebyshevFitArc The arc record with coefficients closest to the current time
    @param callTime The clock time at which the function was called (nanoseconds)
*/
ChebyshevFitArc OEStateEphemAlgorithm::findCurrentArc(const uint64_t callTime) {
    /*! - compute time for fitting interval */
    this->currentEphTime =  (static_cast<double>(callTime) * nanoToSeconds) + this->ephemerisTime - this->vehicleTime;

    /*! - select the fitting coefficients for the nearest fit interval */
    uint32_t nearestArc = 0;
    double smallestTimeDifference = fabs(this->currentEphTime - this->fitCoefficients.at(0).ephemerisTimeMiddle);
    for (auto i = 1; i < MAX_OE_RECORDS; ++i) {
        if (const double timeDifference = fabs(this->currentEphTime - this->fitCoefficients.at(i).ephemerisTimeMiddle);
            timeDifference < smallestTimeDifference) {
            nearestArc = i;
            smallestTimeDifference = timeDifference;
        }
    }

    /*! - determine the scaled fitting time */
    return this->fitCoefficients.at(nearestArc);
}

/*! This method scales the current ephemeris time to a normalized value within the range [-1, 1]
    based on the arc's middle time and time radius. If the scaled value exceeds this range,
    it is clamped to ±1.
    @return double The scaled time value in the range [-1, 1]
    @param arc The Chebyshev fit arc containing the time middle and radius parameters
*/
double OEStateEphemAlgorithm::scaleEphemerisTime(const ChebyshevFitArc& arc) const {
    double currentScaledValue = (this->currentEphTime - arc.ephemerisTimeMiddle) / arc.ephemerisTimeRadius;
    if (fabs(currentScaledValue) > 1) {
        currentScaledValue = currentScaledValue / fabs(currentScaledValue);
    }
    return currentScaledValue;
}

/*! This method evaluates the Chebyshev polynomial coefficients at the given scaled time value
    to determine the orbital elements. It calculates all six orbital elements (semi-major axis,
    eccentricity, inclination, argument of periapsis, RAAN, and true anomaly) from the stored
    Chebyshev coefficients. The method handles different orbit types (elliptic, hyperbolic, parabolic)
    and converts mean anomaly to true anomaly when necessary.
    @return ClassicalElementsF32 The computed classical orbital elements
    @param currentScaledValue The normalized time value in the range [-1, 1]
    @param arc The Chebyshev fit arc containing all coefficient arrays
*/
ClassicalElementsF32 OEStateEphemAlgorithm::evaluateCoefficients(const double currentScaledValue,
                                                                 const ChebyshevFitArc& arc) {
    /* - determine orbit elements from chebychev polynominals */
    ClassicalElementsF32 elements{};
    const double radiusPeriapsis =
        calculateChebyValue(arc.radiusPeriapsisCoefficients, arc.numberChebCoefficients, currentScaledValue) *
        kmToMeters;  // coefficients are in km but module operates in meters
    elements.inclination =
        calculateChebyValueF32(arc.inclinationCoefficients, arc.numberChebCoefficients, static_cast<float>(currentScaledValue));
    elements.eccentricity =
        calculateChebyValueF32(arc.eccentricityCoefficients, arc.numberChebCoefficients, static_cast<float>(currentScaledValue));
    elements.argPeriapsis =
        calculateChebyValueF32(arc.argPeriapsisCoefficients, arc.numberChebCoefficients, static_cast<float>(currentScaledValue));
    elements.rightAscensionAscendingNode =
        calculateChebyValueF32(arc.raanCoefficients, arc.numberChebCoefficients, static_cast<float>(currentScaledValue));
    float const anomalyAngle =
        calculateChebyValueF32(arc.trueAnomalyCoefficients, arc.numberChebCoefficients, static_cast<float>(currentScaledValue));

    /*! - determine the true anomaly angle */
    if (arc.anomalyFlag == 0) {
        elements.trueAnomaly = anomalyAngle;
    } else if (elements.eccentricity < 1.0) {
        /* input is mean elliptic anomaly angle */
        elements.trueAnomaly = OrbitalMotion::meanToTrueAnomalyF32(anomalyAngle, elements.eccentricity);
    } else {
        /* input is mean hyperbolic anomaly angle */
        elements.trueAnomaly = OrbitalMotion::hyperbolicToTrueAnomalyF32(
            OrbitalMotion::meanToHyperbolicAnomalyF32(anomalyAngle, elements.eccentricity), elements.eccentricity);
    }

    /*! - determine semi-major axis */
    if (fabs(elements.eccentricity - 1.0) > toleranceF32) {
        /* elliptic or hyperbolic case */
        elements.semiMajorAxis = radiusPeriapsis / (1.0 - elements.eccentricity);
    } else {
        /* parabolic case, the elem2rv() function assumes a parabola has a = 0 */
        elements.semiMajorAxis = 0.0;
    }
    return elements;
}

/*! This method takes the current time and computes the state of the object
    using that time and the stored Chebyshev coefficients. If the time provided
    is outside the specified range, the position vectors rail high/low appropriately.
    Special handling is included for central bodies (all zero coefficients) which return
    a zero state vector.
    @return CartesianState The computed position and velocity vectors in Cartesian coordinates
    @param callTime The clock time at which the function was called (nanoseconds)
*/
CartesianState OEStateEphemAlgorithm::update(const uint64_t callTime) {
    /*! If all of the radius of periapsis components are zero, this is the central body and should return all zeros*/
    if (std::ranges::all_of(this->fitCoefficients.at(0).radiusPeriapsisCoefficients.begin(),
                            this->fitCoefficients.at(0).radiusPeriapsisCoefficients.end(),
                            [](float val) { return std::abs(val) < tolerance; })) {
        CartesianState outputCartesianState{};
        return outputCartesianState;
    }

    const auto currentArc = this->findCurrentArc(callTime);
    const auto currentScaledValue = this->scaleEphemerisTime(currentArc);
    const auto orbitalElements = evaluateCoefficients(currentScaledValue, currentArc);

    /*! - Determine position and velocity vectors */
    auto cartesianState = OrbitalMotion::elementsToCartesianStateF32(static_cast<float>(this->mu), orbitalElements);

    return cartesianState;
}

/*! This method sets the gravitational parameter of the central body for the orbital calculations.
    @return void
    @param gravitationalParameter The gravitational parameter (m^3/s^2), must be positive
*/
void OEStateEphemAlgorithm::setCentralBodyGravitationalParameter(const double gravitationalParameter) {
    if (gravitationalParameter < 0) {
        FS_THROW_INVALID_ARGUMENT("GravitationalParameter in OEStateEphemAlgorithm must be positive.");
    }
    this->mu = gravitationalParameter;
};

/*! This method retrieves the gravitational parameter of the central body.
    @return double The gravitational parameter (m^3/s^2)
*/
double OEStateEphemAlgorithm::getCentralBodyGravitationalParameter() const { return this->mu; };

/*! This method sets the number of Chebyshev coefficients for a specified arc.
    @return void
    @param arcNumber The index of the arc to modify
    @param numberOfCoefficients The number of coefficients to set for this arc
*/
void OEStateEphemAlgorithm::setArcNumberOfCoefficients(const unsigned int arcNumber,
                                                       const unsigned int numberOfCoefficients) {
    this->fitCoefficients.at(arcNumber).numberChebCoefficients = numberOfCoefficients;
};

/*! This method retrieves the number of Chebyshev coefficients for a specified arc.
    @return unsigned int The number of coefficients for the specified arc
    @param arcNumber The index of the arc to query
*/
unsigned int OEStateEphemAlgorithm::getArcNumberOfCoefficients(const unsigned int arcNumber) const {
    return this->fitCoefficients.at(arcNumber).numberChebCoefficients;
};

/*! This method sets the middle time (ephemeris time at the center of the arc) for a specified arc.
    @return void
    @param arcNumber The index of the arc to modify
    @param timeMiddle The ephemeris time at the arc's midpoint (seconds)
*/
void OEStateEphemAlgorithm::setArcMiddleTime(const unsigned int arcNumber, const double timeMiddle) {
    this->fitCoefficients.at(arcNumber).ephemerisTimeMiddle = timeMiddle;
};

double OEStateEphemAlgorithm::getArcMiddleTime(const unsigned int arcNumber) const {
    /*! This method retrieves the middle time for a specified arc.
        @return float The ephemeris time at the arc's midpoint (seconds)
        @param arcNumber The index of the arc to query
    */
    return this->fitCoefficients.at(arcNumber).ephemerisTimeMiddle;
};

/*! This method sets the time radius (half of the total time range) for a specified arc.
    @return void
    @param arcNumber The index of the arc to modify
    @param timeRadius The time radius for the arc (seconds)
*/
void OEStateEphemAlgorithm::setArcRadiusTime(const unsigned int arcNumber, const double timeRadius) {
    this->fitCoefficients.at(arcNumber).ephemerisTimeRadius = timeRadius;
};

double OEStateEphemAlgorithm::getArcRadiusTime(const unsigned int arcNumber) const {
    /*! This method retrieves the time radius for a specified arc.
        @return float The time radius for the arc (seconds)
        @param arcNumber The index of the arc to query
    */
    return this->fitCoefficients.at(arcNumber).ephemerisTimeRadius;
};

/*! This method sets the anomaly flag for a specified arc. The flag indicates whether the anomaly
    angle is true anomaly (0) or mean anomaly (1).
    @return void
    @param arcNumber The index of the arc to modify
    @param anomalyFlag The anomaly type flag (0 = true anomaly, 1 = mean anomaly)
*/
void OEStateEphemAlgorithm::setArcAnomalyFlag(const unsigned int arcNumber, const unsigned int anomalyFlag) {
    this->fitCoefficients.at(arcNumber).anomalyFlag = anomalyFlag;
};

/*! This method retrieves the anomaly flag for a specified arc.
    @return unsigned int The anomaly type flag (0 = true anomaly, 1 = mean anomaly)
    @param arcNumber The index of the arc to query
*/
unsigned int OEStateEphemAlgorithm::getArcAnomalyFlag(const unsigned int arcNumber) const {
    return this->fitCoefficients.at(arcNumber).anomalyFlag;
};

/*! This method sets the Chebyshev coefficients for the radius at periapsis for a specified arc.
    @return void
    @param arcNumber The index of the arc to modify
    @param radiusPeriapsisCoefficients Array of Chebyshev coefficients for radius at periapsis
*/
void OEStateEphemAlgorithm::setArcRadiusPeriapsisCoefficients(
    const unsigned int arcNumber,
    const std::array<double, MAX_OE_COEFF>& radiusPeriapsisCoefficients) {
    this->fitCoefficients.at(arcNumber).radiusPeriapsisCoefficients = radiusPeriapsisCoefficients;
};

/*! This method retrieves the Chebyshev coefficients for the radius at periapsis for a specified arc.
    @return std::array<double, MAX_OE_COEFF> Array of Chebyshev coefficients for radius at periapsis
    @param arcNumber The index of the arc to query
*/
std::array<double, MAX_OE_COEFF> OEStateEphemAlgorithm::getArcRadiusPeriapsisCoefficients(
    const unsigned int arcNumber) const {
    return this->fitCoefficients.at(arcNumber).radiusPeriapsisCoefficients;
};

/*! This method sets the Chebyshev coefficients for eccentricity for a specified arc.
    @return void
    @param arcNumber The index of the arc to modify
    @param eccentricityCoefficients Array of Chebyshev coefficients for eccentricity
*/
void OEStateEphemAlgorithm::setArcEccentricityCoefficients(
    const unsigned int arcNumber,
    const std::array<float, MAX_OE_COEFF>& eccentricityCoefficients) {
    this->fitCoefficients.at(arcNumber).eccentricityCoefficients = eccentricityCoefficients;
};

/*! This method retrieves the Chebyshev coefficients for eccentricity for a specified arc.
    @return std::array<float, MAX_OE_COEFF> Array of Chebyshev coefficients for eccentricity
    @param arcNumber The index of the arc to query
*/
std::array<float, MAX_OE_COEFF> OEStateEphemAlgorithm::getArcEccentricityCoefficients(
    const unsigned int arcNumber) const {
    return this->fitCoefficients.at(arcNumber).eccentricityCoefficients;
};

/*! This method sets the Chebyshev coefficients for inclination for a specified arc.
    @return void
    @param arcNumber The index of the arc to modify
    @param inclinationCoefficients Array of Chebyshev coefficients for inclination
*/
void OEStateEphemAlgorithm::setArcInclinationCoefficients(
    const unsigned int arcNumber,
    const std::array<float, MAX_OE_COEFF>& inclinationCoefficients) {
    this->fitCoefficients.at(arcNumber).inclinationCoefficients = inclinationCoefficients;
};

/*! This method retrieves the Chebyshev coefficients for inclination for a specified arc.
    @return std::array<float, MAX_OE_COEFF> Array of Chebyshev coefficients for inclination
    @param arcNumber The index of the arc to query
*/
std::array<float, MAX_OE_COEFF> OEStateEphemAlgorithm::getArcInclinationCoefficients(
    const unsigned int arcNumber) const {
    return this->fitCoefficients.at(arcNumber).inclinationCoefficients;
};

/*! This method sets the Chebyshev coefficients for argument of periapsis for a specified arc.
    @return void
    @param arcNumber The index of the arc to modify
    @param argPeriapsisCoefficients Array of Chebyshev coefficients for argument of periapsis
*/
void OEStateEphemAlgorithm::setArcArgPeriapsisCoefficients(
    const unsigned int arcNumber,
    const std::array<float, MAX_OE_COEFF>& argPeriapsisCoefficients) {
    this->fitCoefficients.at(arcNumber).argPeriapsisCoefficients = argPeriapsisCoefficients;
};

/*! This method retrieves the Chebyshev coefficients for argument of periapsis for a specified arc.
    @return std::array<float, MAX_OE_COEFF> Array of Chebyshev coefficients for argument of periapsis
    @param arcNumber The index of the arc to query
*/
std::array<float, MAX_OE_COEFF> OEStateEphemAlgorithm::getArcArgPeriapsisCoefficients(
    const unsigned int arcNumber) const {
    return this->fitCoefficients.at(arcNumber).argPeriapsisCoefficients;
};

/*! This method sets the Chebyshev coefficients for right ascension of the ascending node (RAAN)
    for a specified arc.
    @return void
    @param arcNumber The index of the arc to modify
    @param raanCoefficients Array of Chebyshev coefficients for RAAN
*/
void OEStateEphemAlgorithm::setArcRaanCoefficients(const unsigned int arcNumber,
                                                   const std::array<float, MAX_OE_COEFF>& raanCoefficients) {
    this->fitCoefficients.at(arcNumber).raanCoefficients = raanCoefficients;
};

/*! This method retrieves the Chebyshev coefficients for right ascension of the ascending node (RAAN)
    for a specified arc.
    @return std::array<float, MAX_OE_COEFF> Array of Chebyshev coefficients for RAAN
    @param arcNumber The index of the arc to query
*/
std::array<float, MAX_OE_COEFF> OEStateEphemAlgorithm::getArcRaanCoefficients(const unsigned int arcNumber) const {
    return this->fitCoefficients.at(arcNumber).raanCoefficients;
};

/*! This method sets the Chebyshev coefficients for true anomaly for a specified arc.
    @return void
    @param arcNumber The index of the arc to modify
    @param trueAnomalyCoefficients Array of Chebyshev coefficients for true anomaly
*/
void OEStateEphemAlgorithm::setArcTrueAnomalyCoefficients(
    const unsigned int arcNumber,
    const std::array<float, MAX_OE_COEFF>& trueAnomalyCoefficients) {
    this->fitCoefficients.at(arcNumber).trueAnomalyCoefficients = trueAnomalyCoefficients;
};

/*! This method retrieves the Chebyshev coefficients for true anomaly for a specified arc.
    @return std::array<float, MAX_OE_COEFF> Array of Chebyshev coefficients for true anomaly
    @param arcNumber The index of the arc to query
*/
std::array<float, MAX_OE_COEFF> OEStateEphemAlgorithm::getArcTrueAnomalyCoefficients(
    const unsigned int arcNumber) const {
    return this->fitCoefficients.at(arcNumber).trueAnomalyCoefficients;
};

/*! This method sets the ephemeris time offset referenced to J2000 epoch.
    @return void
    @param time The ephemeris time offset (seconds)
*/
void OEStateEphemAlgorithm::setEphemerisTimeJ2000(double time) { this->ephemerisTime = time; }

/*! This method retrieves the ephemeris time offset referenced to J2000 epoch.
    @return double The ephemeris time offset (seconds)
*/
double OEStateEphemAlgorithm::getEphemerisTimeJ2000() const { return this->ephemerisTime; }

/*! This method sets the vehicle time offset used in ephemeris calculations.
    @return void
    @param time The vehicle time offset (seconds)
*/
void OEStateEphemAlgorithm::setVehicleTime(double time) { this->vehicleTime = time; }

/*! This method retrieves the vehicle time offset used in ephemeris calculations.
    @return double The vehicle time offset (seconds)
*/
double OEStateEphemAlgorithm::getVehicleTime() const { return this->vehicleTime; }
