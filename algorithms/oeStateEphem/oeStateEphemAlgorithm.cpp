#include "oeStateEphemAlgorithm.h"

#include "utilities/fsw/chebyshevUtilities.h"
#include "utilities/fsw/timeConstants.h"
#include <algorithm>

OEStateEphemAlgorithm::OEStateEphemAlgorithm(const OEStateEphemConfig& config) : cfg(config) { setConfig(config); }

void OEStateEphemAlgorithm::setConfig(const OEStateEphemConfig& config) { this->cfg = config; }

/*! This method selects the Chebyshev fit arc whose middle time is closest to the supplied ephemeris time.
    @return ChebyshevFitArc The arc record with coefficients closest to the current time
    @param currentEphTime The ephemeris time to search against (seconds)
*/
ChebyshevFitArc OEStateEphemAlgorithm::findCurrentArc(const double currentEphTime) const {
    const auto& fitCoefficients = this->cfg.getFitCoefficients();
    uint32_t nearestArc = 0;
    double smallestTimeDifference = fabs(currentEphTime - fitCoefficients.at(0).ephemerisTimeMiddle);
    for (unsigned int i = 1; i < this->cfg.getNumberOfArcs(); ++i) {
        const double timeDifference = fabs(currentEphTime - fitCoefficients.at(i).ephemerisTimeMiddle);
        if (timeDifference < smallestTimeDifference) {
            nearestArc = i;
            smallestTimeDifference = timeDifference;
        }
    }
    return fitCoefficients.at(nearestArc);
}

/*! This method scales the supplied ephemeris time to a normalized value within [-1, 1] based on the arc's middle
    time and time radius. If the scaled value exceeds this range, it is clamped to +/-1.
    @return double The scaled time value in the range [-1, 1]
    @param arc The Chebyshev fit arc containing the time middle and radius parameters
    @param currentEphTime The ephemeris time to scale (seconds)
*/
double OEStateEphemAlgorithm::scaleEphemerisTime(const ChebyshevFitArc& arc, const double currentEphTime) {
    double currentScaledValue = 0;
    if (arc.ephemerisTimeRadius > kTolerance) {
        currentScaledValue = (currentEphTime - arc.ephemerisTimeMiddle) / arc.ephemerisTimeRadius;
        if (fabs(currentScaledValue) > 1.0F) {
            currentScaledValue = currentScaledValue / fabs(currentScaledValue);
        }
    }
    return currentScaledValue;
}

/*! This method evaluates the Chebyshev polynomial coefficients at the given scaled time value
    to determine the orbital elements. It calculates all six orbital elements (semi-major axis,
    eccentricity, inclination, argument of periapsis, RAAN, and true anomaly) from the stored
    Chebyshev coefficients. The method handles different orbit types (elliptic, hyperbolic, parabolic)
    and converts mean anomaly to true anomaly when necessary.
    @return ClassicalElements The computed classical orbital elements
    @param currentScaledValue The normalized time value in the range [-1, 1]
    @param arc The Chebyshev fit arc containing all coefficient arrays
*/
orbitalMotion::ClassicalElements OEStateEphemAlgorithm::evaluateCoefficients(const double currentScaledValue,
                                                                             const ChebyshevFitArc& arc) {
    /* - determine orbit elements from chebychev polynominals */
    orbitalMotion::ClassicalElements elements{};
    const double radiusPeriapsis =
        calculateChebyValue(arc.radiusPeriapsisCoefficients, arc.numberChebCoefficients, currentScaledValue);
    elements.inclination =
        calculateChebyValue(arc.inclinationCoefficients, arc.numberChebCoefficients, currentScaledValue);
    elements.eccentricity =
        calculateChebyValue(arc.eccentricityCoefficients, arc.numberChebCoefficients, currentScaledValue);
    elements.argPeriapsis =
        calculateChebyValue(arc.argPeriapsisCoefficients, arc.numberChebCoefficients, currentScaledValue);
    elements.rightAscensionAscendingNode =
        calculateChebyValue(arc.raanCoefficients, arc.numberChebCoefficients, currentScaledValue);
    const double anomalyAngle =
        calculateChebyValue(arc.trueAnomalyCoefficients, arc.numberChebCoefficients, currentScaledValue);

    /*! - determine the true anomaly angle */
    if (arc.anomalyFlag == AnomalyType::TRUE_ANOMALY) {
        elements.trueAnomaly = anomalyAngle;
    } else if (elements.eccentricity < 1.0) {
        /* input is mean elliptic anomaly angle */
        elements.trueAnomaly = orbitalMotion::meanToTrueAnomaly(anomalyAngle, elements.eccentricity);
    } else {
        /* input is mean hyperbolic anomaly angle */
        elements.trueAnomaly = orbitalMotion::hyperbolicToTrueAnomaly(
            orbitalMotion::meanToHyperbolicAnomaly(anomalyAngle, elements.eccentricity), elements.eccentricity);
    }

    /*! - determine semi-major axis */
    if (fabs(elements.eccentricity - 1.0) > kTolerance) {
        /* elliptic or hyperbolic case */
        elements.semiMajorAxis = radiusPeriapsis / (1.0 - elements.eccentricity);
    } else {
        /* parabolic case, the elem2rv() function assumes a parabola has a = 0 */
        elements.semiMajorAxis = 0.0;
    }
    return elements;
}

/*! Check if all the radius of periapsis lead coefficients are zero (a central body returns a zero state).
    @return bool
*/
bool OEStateEphemAlgorithm::allParametersNull() const {
    bool allZero = true;
    for (const auto& arc : this->cfg.getFitCoefficients()) {
        if (fabs(arc.radiusPeriapsisCoefficients.at(0)) >= kTolerance) {
            allZero = false;
        }
    }
    return allZero;
}

/*! This method takes the current time and computes the state of the object using that time and the stored
    Chebyshev coefficients. If the time provided is outside the specified range, the position vectors rail
    high/low appropriately. Central bodies (all zero coefficients) return a zero state vector.
    @return CartesianState The computed position and velocity vectors in Cartesian coordinates
    @param callTime The clock time at which the function was called (nanoseconds)
*/
orbitalMotion::CartesianState OEStateEphemAlgorithm::update(const uint64_t callTime) const {
    orbitalMotion::CartesianState outputCartesianState{};
    outputCartesianState.position = Eigen::Vector3d::Zero();
    outputCartesianState.velocity = Eigen::Vector3d::Zero();

    /*! Only evaluate the fit when this is not a central body (central bodies have all radius-of-periapsis
        coefficients zero and keep the zero-initialized state). */
    if (!this->allParametersNull()) {
        /*! - compute time for fitting interval, clamped to be non-negative */
        double currentEphTime = (static_cast<double>(callTime) * kNano2Sec) + this->cfg.getEphemerisTimeJ2000() -
                                this->cfg.getVehicleTimeOffset();
        currentEphTime = std::max<double>(currentEphTime, 0);

        const auto currentArc = this->findCurrentArc(currentEphTime);
        const auto currentScaledValue = scaleEphemerisTime(currentArc, currentEphTime);
        const auto orbitalElements = evaluateCoefficients(currentScaledValue, currentArc);

        /*! - Determine position and velocity vectors */
        outputCartesianState =
            orbitalMotion::elementsToCartesianState(this->cfg.getCentralBodyGravitationalParameter(), orbitalElements);
    }

    return outputCartesianState;
}
