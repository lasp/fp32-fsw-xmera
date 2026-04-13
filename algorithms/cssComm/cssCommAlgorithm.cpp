#include "cssCommAlgorithm.h"
#include "utilities/chebyshevUtilities.h"
#include "utilities/freestandingInvalidArgument.h"
#include <algorithm>

/*! This method takes the raw sensor data from the coarse sun sensors and
 converts that information to the format used by the CSS nav.
 @return void
 @param inputValues [-] Current measured CSS value for the constellation of CSS sensors
 */
std::array<double, kMaxNumCssSensors> CssCommAlgorithm::update(
    const std::array<double, kMaxNumCssSensors>& inputValues) const {
    std::array<double, kMaxNumCssSensors> outputValues{};

    for (uint32_t i = 0; i < this->numSensors; ++i) {
        double const measuredValue = inputValues.at(i) / this->maxSensorValue; /* Scale Sensor Data */

        /* Calculate correction using Chebyshev polynomial */
        double const correction = calculateChebyValue(this->chebyPolynomials, this->chebyCount, measuredValue);

        double correctedValue = measuredValue + correction;

        if (correctedValue > 1.0) {
            correctedValue = 1.0;
        } else if (correctedValue < 0.0) {
            correctedValue = 0.0;
        }
        outputValues.at(i) = correctedValue;
    }

    return outputValues;
}

/*! Set the number of CSS sensors
 @return void
 @param numberOfSensors [-] number of CSS sensors
*/
void CssCommAlgorithm::setNumSensors(const uint32_t numberOfSensors) {
    if (numberOfSensors > kMaxNumCssSensors) {
        FS_THROW_INVALID_ARGUMENT("The configured number of CSS sensors exceeds the maximum");
    }
    if (numberOfSensors <= 0) {
        FSW_THROW_INVALID_ARGUMENT("The number of configures CSS sensors must be positive.");
    }
    this->numSensors = numberOfSensors;
}

/*! Get the number of CSS sensors
 @return uint32_t
*/
uint32_t CssCommAlgorithm::getNumSensors() const { return this->numSensors; }

/*! Set the maximum sensor value
 @return void
 @param maxValue [-] maximum sensor value
*/
void CssCommAlgorithm::setMaxSensorValue(const double maxValue) {
    if (maxValue <= 0) {
        FSW_THROW_INVALID_ARGUMENT(
            "The maximum CSS sensor value must be positive. Otherwise, CSS sensor values "
            "will be normalized by zero, inducing faux saturation!");
    }
    this->maxSensorValue = maxValue;
}

/*! Get the maximum sensor value
 @return double
*/
double CssCommAlgorithm::getMaxSensorValue() const { return this->maxSensorValue; }

/*! Set the cheby polynomial count
 @return void
 @param count [-] cheby polynomial count
*/
void CssCommAlgorithm::setChebyCount(const uint32_t count) {
    if (count <= 0) {
        FSW_THROW_INVALID_ARGUMENT("The cheby polynomial count must be positive.");
    }
    if (count > static_cast<uint32_t>(kMaxNumChebyPolys)) {
        FSW_THROW_INVALID_ARGUMENT("The cheby polynomial count exceeds the maximum allowed.");
    }
    this->chebyCount = count;
}

/*! Get the cheby polynomial count
 @return uint32_t
*/
uint32_t CssCommAlgorithm::getChebyCount() const { return this->chebyCount; }

/*! Set the cheby polynomials
 @return void
 @param polynomials [-] cheby polynomials
*/
void CssCommAlgorithm::setChebyPolynomials(const std::array<double, kMaxNumChebyPolys>& polynomials) {
    this->chebyPolynomials = polynomials;
}

/*! Get the cheby polynomials
 @return std::array<double, kMaxNumChebyPolys>
*/
std::array<double, kMaxNumChebyPolys> CssCommAlgorithm::getChebyPolynomials() const { return this->chebyPolynomials; }
