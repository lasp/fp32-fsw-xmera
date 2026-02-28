/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#include "oeStateEphemAlgorithm.h"
#include "../freestandingInvalidArgument.h"
#include "utilities/ephemerisUtilities.h"
ChebyshevFitArc OEStateEphemAlgorithm::findCurrentArc(const uint64_t callTime) {
    /*! - compute time for fitting interval */
    this->currentEphTime = callTime * 1e-9 + this->ephemerisTime - this->vehicleTime;

    /*! - select the fitting coefficients for the nearest fit interval */
    uint32_t nearestArc = 0;
    double smallestTimeDifference = fabs(this->currentEphTime - this->fitCoefficients.at(0).ephemerisTimeMiddle);
    for (auto i = 1; i < MAX_OE_RECORDS; ++i) {
        double timeDifference = fabs(this->currentEphTime - this->fitCoefficients.at(i).ephemerisTimeMiddle);
        if (timeDifference < smallestTimeDifference) {
            nearestArc = i;
            smallestTimeDifference = timeDifference;
        }
    }

    /*! - determine the scaled fitting time */
    return this->fitCoefficients.at(nearestArc);
}

double OEStateEphemAlgorithm::scaleEphemerisTime(const ChebyshevFitArc& arc) const {
    double currentScaledValue = (this->currentEphTime - arc.ephemerisTimeMiddle) / arc.ephemerisTimeRadius;
    if (fabs(currentScaledValue) > 1.0) {
        currentScaledValue = currentScaledValue / fabs(currentScaledValue);
    }
    return currentScaledValue;
}

ClassicalElementsF32 OEStateEphemAlgorithm::evaluateCoefficients(const double currentScaledValue,
                                                                 const ChebyshevFitArc& arc) {
    /* - determine orbit elements from chebychev polynominals */
    float anomalyAngle{}; /* [r] general anomaly angle variable */
    ClassicalElementsF32 elements{};
    const double radiusPeriapsis =
        calculateChebyValue(arc.radiusPeriapsisCoefficients.data(), arc.numberChebCoefficients, currentScaledValue) *
        1e3;  // coefficients are in km but module operates in meters
    elements.inclination =
        calculateChebyValueF32(arc.inclinationCoefficients.data(), arc.numberChebCoefficients, currentScaledValue);
    elements.eccentricity =
        calculateChebyValueF32(arc.eccentricityCoefficients.data(), arc.numberChebCoefficients, currentScaledValue);
    elements.argPeriapsis =
        calculateChebyValueF32(arc.argPeriapsisCoefficients.data(), arc.numberChebCoefficients, currentScaledValue);
    elements.rightAscensionAscendingNode =
        calculateChebyValueF32(arc.raanCoefficients.data(), arc.numberChebCoefficients, currentScaledValue);
    anomalyAngle =
        calculateChebyValueF32(arc.trueAnomalyCoefficients.data(), arc.numberChebCoefficients, currentScaledValue);

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
    if (fabs(elements.eccentricity - 1.0) > 1e-6) {
        /* elliptic or hyperbolic case */
        elements.semiMajorAxis = radiusPeriapsis / (1.0 - elements.eccentricity);
    } else {
        /* parabolic case, the elem2rv() function assumes a parabola has a = 0 */
        elements.semiMajorAxis = 0.0;
    }
    return elements;
}

/*! This method takes the current time and computes the state of the object
    using that time and the stored Chebyshev coefficients.  If the time provided
    is outside the specified range, the position vectors rail high/low appropriately.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
EphemerisMsgF32Payload OEStateEphemAlgorithm::updateState(const uint64_t callTime) {
    auto ephmerisMessageOutput = EphemerisMsgF32Payload{};
    /*! - Write the output message time */
    ephmerisMessageOutput.timeTag = callTime * 1e-9;
CartesianState OEStateEphemAlgorithm::update(const uint64_t callTime) {
    /*! If all of the radius of periapsis components are zero, this is the central body and should return all zeros*/
    if (std::all_of(this->fitCoefficients[0].radiusPeriapsisCoefficients.begin(),
                    this->fitCoefficients[0].radiusPeriapsisCoefficients.end(),
                    [](double val) { return std::abs(val) < 1e-10; })) {
        return ephmerisMessageOutput;
    }

    const auto currentArc = this->findCurrentArc(callTime);
    const auto currentScaledValue = this->scaleEphemerisTime(currentArc);
    const auto orbitalElements = evaluateCoefficients(currentScaledValue, currentArc);

    /*! - Determine position and velocity vectors */
    auto cartesianState = OrbitalMotion::elementsToCartesianStateF32(this->gravitationalParameter, orbitalElements);

    return cartesianState;
}

void OEStateEphemAlgorithm::setCentralBodyGravitationalParameter(const float mu) {
    if (mu < 0) {
        FS_THROW_INVALID_ARGUMENT("GravitationalParameter in OEStateEphemAlgorithm must be positive.");
    }
    this->gravitationalParameter = mu;
};

float OEStateEphemAlgorithm::getCentralBodyGravitationalParameter() const { return this->gravitationalParameter; };

void OEStateEphemAlgorithm::setArcNumberOfCoefficients(const unsigned int arcNumber,
                                                       const unsigned int numberOfCoefficients) {
    this->fitCoefficients.at(arcNumber).numberChebCoefficients = numberOfCoefficients;
};

unsigned int OEStateEphemAlgorithm::getArcNumberOfCoefficients(const unsigned int arcNumber) const {
    return this->fitCoefficients.at(arcNumber).numberChebCoefficients;
};

void OEStateEphemAlgorithm::setArcMiddleTime(const unsigned int arcNumber, const double timeMiddle) {
    this->fitCoefficients.at(arcNumber).ephemerisTimeMiddle = timeMiddle;
};

double OEStateEphemAlgorithm::getArcMiddleTime(const unsigned int arcNumber) const {
    return this->fitCoefficients.at(arcNumber).ephemerisTimeMiddle;
};

void OEStateEphemAlgorithm::setArcRadiusTime(const unsigned int arcNumber, const double timeRadius) {
    this->fitCoefficients.at(arcNumber).ephemerisTimeRadius = timeRadius;
};

double OEStateEphemAlgorithm::getArcRadiusTime(const unsigned int arcNumber) const {
    return this->fitCoefficients.at(arcNumber).ephemerisTimeRadius;
};

void OEStateEphemAlgorithm::setArcAnomalyFlag(const unsigned int arcNumber, const unsigned int anomalyFlag) {
    this->fitCoefficients.at(arcNumber).anomalyFlag = anomalyFlag;
};

unsigned int OEStateEphemAlgorithm::getArcAnomalyFlag(unsigned int arcNumber) const {
    return this->fitCoefficients.at(arcNumber).anomalyFlag;
};

void OEStateEphemAlgorithm::setArcRadiusPeriapsisCoefficients(
    const unsigned int arcNumber,
    const std::array<double, MAX_OE_COEFF>& radiusPeriapsisCoefficients) {
    this->fitCoefficients.at(arcNumber).radiusPeriapsisCoefficients = radiusPeriapsisCoefficients;
};

std::array<double, MAX_OE_COEFF> OEStateEphemAlgorithm::getArcRadiusPeriapsisCoefficients(
    const unsigned int arcNumber) {
    return this->fitCoefficients.at(arcNumber).radiusPeriapsisCoefficients;
};

void OEStateEphemAlgorithm::setArcEccentricityCoefficients(
    const unsigned int arcNumber,
    const std::array<float, MAX_OE_COEFF>& eccentricityCoefficients) {
    this->fitCoefficients.at(arcNumber).eccentricityCoefficients = eccentricityCoefficients;
};

std::array<float, MAX_OE_COEFF> OEStateEphemAlgorithm::getArcEccentricityCoefficients(const unsigned int arcNumber) {
    return this->fitCoefficients.at(arcNumber).eccentricityCoefficients;
};

void OEStateEphemAlgorithm::setArcInclinationCoefficients(
    const unsigned int arcNumber,
    const std::array<float, MAX_OE_COEFF>& inclinationCoefficients) {
    this->fitCoefficients.at(arcNumber).inclinationCoefficients = inclinationCoefficients;
};

std::array<float, MAX_OE_COEFF> OEStateEphemAlgorithm::getArcInclinationCoefficients(const unsigned int arcNumber) {
    return this->fitCoefficients.at(arcNumber).inclinationCoefficients;
};

void OEStateEphemAlgorithm::setArcArgPeriapsisCoefficients(
    const unsigned int arcNumber,
    const std::array<float, MAX_OE_COEFF>& argPeriapsisCoefficients) {
    this->fitCoefficients.at(arcNumber).argPeriapsisCoefficients = argPeriapsisCoefficients;
};

std::array<float, MAX_OE_COEFF> OEStateEphemAlgorithm::getArcArgPeriapsisCoefficients(const unsigned int arcNumber) {
    return this->fitCoefficients.at(arcNumber).argPeriapsisCoefficients;
};

void OEStateEphemAlgorithm::setArcRaanCoefficients(const unsigned int arcNumber,
                                                   const std::array<float, MAX_OE_COEFF>& raanCoefficients) {
    this->fitCoefficients.at(arcNumber).raanCoefficients = raanCoefficients;
};

std::array<float, MAX_OE_COEFF> OEStateEphemAlgorithm::getArcRaanCoefficients(const unsigned int arcNumber) {
    return this->fitCoefficients.at(arcNumber).raanCoefficients;
};

void OEStateEphemAlgorithm::setArcTrueAnomalyCoefficients(
    const unsigned int arcNumber,
    const std::array<float, MAX_OE_COEFF>& trueAnomalyCoefficients) {
    this->fitCoefficients.at(arcNumber).trueAnomalyCoefficients = trueAnomalyCoefficients;
};

std::array<float, MAX_OE_COEFF> OEStateEphemAlgorithm::getArcTrueAnomalyCoefficients(const unsigned int arcNumber) {
    return this->fitCoefficients.at(arcNumber).trueAnomalyCoefficients;
};

/*! This method sets the ephemeris and vehicle time offset referenced to J2000 epoch.
    @return void
    @param ephemerisJ2000 The ephemeris time offset (seconds)
    @param vehicleTimeOffset The vehicle time offset (seconds)
*/
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
