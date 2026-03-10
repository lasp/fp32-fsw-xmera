/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#include "oeStateEphemAlgorithm.h"

#include "../freestandingInvalidArgument.h"
#include "utilities/chebyshevUtilities.h"
#include <algorithm>

/*! This method finds the Chebyshev fit arc that is closest in time to the current ephemeris time.
    It computes the current ephemeris time from the call time, ephemeris time offset, and vehicle time,
    then searches through all available arcs to find the one with the smallest time difference.
    @return ChebyshevFitArc The arc record with coefficients closest to the current time
    @param callTime The clock time at which the function was called (nanoseconds)
*/
ChebyshevFitArc OEStateEphemAlgorithm::findCurrentArc(const uint64_t callTime) {
    /*! - compute time for fitting interval */
    this->currentEphTime = (static_cast<double>(callTime) * nanoToSeconds) + this->ephemerisTime - this->vehicleTime;
    this->currentEphTime = std::max<double>(this->currentEphTime, 0);
    /*! - select the fitting coefficients for the nearest fit interval */
    uint32_t nearestArc = 0;
    double smallestTimeDifference = fabs(this->currentEphTime - this->fitCoefficients.at(0).ephemerisTimeMiddle);
    for (unsigned int i = 1; i < kMaxOeRecords; ++i) {
        const double timeDifference = fabs(this->currentEphTime - this->fitCoefficients.at(i).ephemerisTimeMiddle);
        if (timeDifference < smallestTimeDifference) {
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
    if (fabs(currentScaledValue) > 1.0F) {
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
    elements.inclination = calculateChebyValue(
        arc.inclinationCoefficients, arc.numberChebCoefficients, static_cast<float>(currentScaledValue));
    elements.eccentricity = calculateChebyValue(
        arc.eccentricityCoefficients, arc.numberChebCoefficients, static_cast<float>(currentScaledValue));
    elements.argPeriapsis = calculateChebyValue(
        arc.argPeriapsisCoefficients, arc.numberChebCoefficients, static_cast<float>(currentScaledValue));
    elements.rightAscensionAscendingNode =
        calculateChebyValue(arc.raanCoefficients, arc.numberChebCoefficients, static_cast<float>(currentScaledValue));
    const float anomalyAngle = calculateChebyValue(
        arc.trueAnomalyCoefficients, arc.numberChebCoefficients, static_cast<float>(currentScaledValue));

    /*! - determine the true anomaly angle */
    if (arc.anomalyFlag == AnomalyType::TRUE_ANOMALY) {
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
    if (fabs(elements.eccentricity - 1.0) > tolerance) {
        /* elliptic or hyperbolic case */
        elements.semiMajorAxis = radiusPeriapsis / (1.0 - elements.eccentricity);
    } else {
        /* parabolic case, the elem2rv() function assumes a parabola has a = 0 */
        elements.semiMajorAxis = 0.0;
    }
    return elements;
}

/*! Check if all the elements in the radius of periapsis coefficients are zero.
    @return bool
*/
bool OEStateEphemAlgorithm::allParametersNull() const {
    bool allZero = true;
    for (const auto& arc : this->fitCoefficients) {
        if (std::abs(arc.radiusPeriapsisCoefficients.at(0)) >= tolerance) {
            allZero = false;
        }
    }
    return allZero;
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
    CartesianState outputCartesianState{};
    outputCartesianState.position = Eigen::Vector3d::Zero();
    outputCartesianState.velocity = Eigen::Vector3d::Zero();
    /*! If all of the radius of periapsis components are zero, this is the central body and should return all zeros*/
    if (this->allParametersNull()) {
        return outputCartesianState;
    }

    const auto currentArc = this->findCurrentArc(callTime);
    const auto currentScaledValue = this->scaleEphemerisTime(currentArc);
    const auto orbitalElements = evaluateCoefficients(currentScaledValue, currentArc);

    /*! - Determine position and velocity vectors */
    outputCartesianState = OrbitalMotion::elementsToCartesianStateF32(this->mu, orbitalElements);

    return outputCartesianState;
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
    if (numberOfCoefficients < 1) {
        FS_THROW_INVALID_ARGUMENT("numberOfCoefficients in OEStateEphemAlgorithm must be positive.");
    }
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
    if (timeMiddle <= 0) {
        FS_THROW_INVALID_ARGUMENT("arc middle time in OEStateEphemAlgorithm must be positive.");
    }
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
    if (timeRadius <= 0) {
        FS_THROW_INVALID_ARGUMENT("arc radius in OEStateEphemAlgorithm must be strictly positive.");
    }
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
    angle is true anomaly or mean anomaly.
    @return void
    @param arcNumber The index of the arc to modify
    @param anomalyFlag The anomaly type (AnomalyType::TRUE_ANOMALY or AnomalyType::MEAN_ANOMALY)
*/
void OEStateEphemAlgorithm::setArcAnomalyFlag(const unsigned int arcNumber, const AnomalyType& anomalyFlag) {
    this->fitCoefficients.at(arcNumber).anomalyFlag = anomalyFlag;
};

/*! This method retrieves the anomaly flag for a specified arc.
    @return AnomalyType The anomaly type (TRUE_ANOMALY or MEAN_ANOMALY)
    @param arcNumber The index of the arc to query
*/
AnomalyType OEStateEphemAlgorithm::getArcAnomalyFlag(const unsigned int arcNumber) const {
    return this->fitCoefficients.at(arcNumber).anomalyFlag;
};

/*! This method sets the Chebyshev coefficients for the radius at periapsis for a specified arc.
    @return void
    @param arcNumber The index of the arc to modify
    @param radiusPeriapsisCoefficients Array of Chebyshev coefficients for radius at periapsis
*/
void OEStateEphemAlgorithm::setArcRadiusPeriapsisCoefficients(
    const unsigned int arcNumber,
    const std::array<double, kMaxOeCoeff>& radiusPeriapsisCoefficients) {
    this->fitCoefficients.at(arcNumber).radiusPeriapsisCoefficients = radiusPeriapsisCoefficients;
};

/*! This method retrieves the Chebyshev coefficients for the radius at periapsis for a specified arc.
    @return std::array<double, kMaxOeCoeff> Array of Chebyshev coefficients for radius at periapsis
    @param arcNumber The index of the arc to query
*/
std::array<double, kMaxOeCoeff> OEStateEphemAlgorithm::getArcRadiusPeriapsisCoefficients(
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
    const std::array<float, kMaxOeCoeff>& eccentricityCoefficients) {
    this->fitCoefficients.at(arcNumber).eccentricityCoefficients = eccentricityCoefficients;
};

/*! This method retrieves the Chebyshev coefficients for eccentricity for a specified arc.
    @return std::array<float, kMaxOeCoeff> Array of Chebyshev coefficients for eccentricity
    @param arcNumber The index of the arc to query
*/
std::array<float, kMaxOeCoeff> OEStateEphemAlgorithm::getArcEccentricityCoefficients(
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
    const std::array<float, kMaxOeCoeff>& inclinationCoefficients) {
    this->fitCoefficients.at(arcNumber).inclinationCoefficients = inclinationCoefficients;
};

/*! This method retrieves the Chebyshev coefficients for inclination for a specified arc.
    @return std::array<float, kMaxOeCoeff> Array of Chebyshev coefficients for inclination
    @param arcNumber The index of the arc to query
*/
std::array<float, kMaxOeCoeff> OEStateEphemAlgorithm::getArcInclinationCoefficients(
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
    const std::array<float, kMaxOeCoeff>& argPeriapsisCoefficients) {
    this->fitCoefficients.at(arcNumber).argPeriapsisCoefficients = argPeriapsisCoefficients;
};

/*! This method retrieves the Chebyshev coefficients for argument of periapsis for a specified arc.
    @return std::array<float, kMaxOeCoeff> Array of Chebyshev coefficients for argument of periapsis
    @param arcNumber The index of the arc to query
*/
std::array<float, kMaxOeCoeff> OEStateEphemAlgorithm::getArcArgPeriapsisCoefficients(
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
                                                   const std::array<float, kMaxOeCoeff>& raanCoefficients) {
    this->fitCoefficients.at(arcNumber).raanCoefficients = raanCoefficients;
};

/*! This method retrieves the Chebyshev coefficients for right ascension of the ascending node (RAAN)
    for a specified arc.
    @return std::array<float, kMaxOeCoeff> Array of Chebyshev coefficients for RAAN
    @param arcNumber The index of the arc to query
*/
std::array<float, kMaxOeCoeff> OEStateEphemAlgorithm::getArcRaanCoefficients(const unsigned int arcNumber) const {
    return this->fitCoefficients.at(arcNumber).raanCoefficients;
};

/*! This method sets the Chebyshev coefficients for true anomaly for a specified arc.
    @return void
    @param arcNumber The index of the arc to modify
    @param trueAnomalyCoefficients Array of Chebyshev coefficients for true anomaly
*/
void OEStateEphemAlgorithm::setArcTrueAnomalyCoefficients(
    const unsigned int arcNumber,
    const std::array<float, kMaxOeCoeff>& trueAnomalyCoefficients) {
    this->fitCoefficients.at(arcNumber).trueAnomalyCoefficients = trueAnomalyCoefficients;
};

/*! This method retrieves the Chebyshev coefficients for true anomaly for a specified arc.
    @return std::array<float, kMaxOeCoeff> Array of Chebyshev coefficients for true anomaly
    @param arcNumber The index of the arc to query
*/
std::array<float, kMaxOeCoeff> OEStateEphemAlgorithm::getArcTrueAnomalyCoefficients(
    const unsigned int arcNumber) const {
    return this->fitCoefficients.at(arcNumber).trueAnomalyCoefficients;
};

/*! This method sets the ephemeris and vehicle time offset referenced to J2000 epoch.
    @return void
    @param ephemerisJ2000 The ephemeris time offset (seconds)
    @param vehicleTimeOffset The vehicle time offset (seconds)
*/
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
void OEStateEphemAlgorithm::setModuleTime(const double ephemerisJ2000, const double vehicleTimeOffset) {
    if (ephemerisJ2000 < 0) {
        FS_THROW_INVALID_ARGUMENT("EphemerisJ2000 time in OEStateEphemAlgorithm must be positive.");
    }
    if (vehicleTimeOffset > ephemerisJ2000) {
        FS_THROW_INVALID_ARGUMENT("vehicleTime in OEStateEphemAlgorithm must be greater than ephemerisJ2000 time.");
    }
    this->ephemerisTime = ephemerisJ2000;
    this->vehicleTime = vehicleTimeOffset;
}

/*! This method retrieves the ephemeris time offset referenced to J2000 epoch.
    @return double The ephemeris time offset (seconds)
*/
double OEStateEphemAlgorithm::getEphemerisTimeJ2000() const { return this->ephemerisTime; }

/*! This method retrieves the vehicle time offset used in ephemeris calculations.
    @return double The vehicle time offset (seconds)
*/
double OEStateEphemAlgorithm::getVehicleTime() const { return this->vehicleTime; }
