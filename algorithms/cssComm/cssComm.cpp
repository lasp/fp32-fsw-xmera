#include "cssComm.h"
#include "utilities/chebyshevUtilities.h"
#include <algorithm>
#include <stdexcept>

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void CssComm::reset(uint64_t callTime) {
    // check if the required message has not been connected
    if (!this->sensorListInMsg.isLinked()) {
        throw std::invalid_argument("cssComm.sensorListInMsg wasn't connected.");
    }
}

/*! This method takes the raw sensor data from the coarse sun sensors and
 converts that information to the format used by the CSS nav.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void CssComm::updateState(uint64_t callTime) {
    std::array<double, MAX_NUM_CSS_SENSORS> inputValues{}; /* [-] Current measured CSS value for the constellation of CSS sensors */
    std::array<double, MAX_NUM_CSS_SENSORS> outputValues{};

    // read sensor list input msg
    CSSArraySensorMsgPayload inMsgBuffer = this->sensorListInMsg();
    std::ranges::copy(std::ranges::begin(inMsgBuffer.CosValue), std::ranges::end(inMsgBuffer.CosValue), inputValues.begin());

    /*! - Loop over the sensors and compute data
         -# Check appropriate range on sensor and calibrate
         -# If Chebyshev polynomials are configured:
             - Seed polynominal computations
             - Loop over polynominals to compute estimated correction factor
             - Output is base value plus the correction factor
         -# If sensor output range is incorrect, set output value to zero
     */
    for (uint32_t i = 0; i < this->numSensors; ++i) {
        double measuredValue = inputValues[i] / this->maxSensorValue; /* Scale Sensor Data */

        /* Calculate correction using Chebyshev polynomial */
        double correction = calculateChebyValue(this->chebyPolynomials, this->chebyCount, measuredValue);

        outputValues[i] = measuredValue + correction;

        if (outputValues[i] > 1.0) {
            outputValues[i] = 1.0;
        } else if (outputValues[i] < 0.0) {
            outputValues[i] = 0.0;
        }
    }

    CSSArraySensorMsgPayload outputBuffer{};
    std::ranges::copy(outputValues, outputBuffer.CosValue);

    /*! - Write aggregate output into output message */
    this->cssArrayOutMsg.write(&outputBuffer, this->moduleID, callTime);
}

/*! Set the number of CSS sensors
 @return void
 @param numberOfSensors [-] number of CSS sensors
*/
void CssComm::setNumSensors(const uint32_t numberOfSensors) {
    if (numberOfSensors > MAX_NUM_CSS_SENSORS) {
        throw std::invalid_argument("The configured number of CSS sensors exceeds the maximum");
    }
    if (numberOfSensors <= 0) {
        throw std::invalid_argument("The number of configures CSS sensors must be positive.");
    }
    this->numSensors = numberOfSensors;
}

/*! Get the number of CSS sensors
 @return uint32_t
*/
uint32_t CssComm::getNumSensors() const { return this->numSensors; }

/*! Set the maximum sensor value
 @return void
 @param maxValue [-] maximum sensor value
*/
void CssComm::setMaxSensorValue(const double maxValue) {
    if (maxValue <= 0) {
        throw std::invalid_argument("The maximum CSS sensor value must be positive. Otherwise, CSS sensor values "
                                    "will be normalized by zero, inducing faux saturation!");
    }
    this->maxSensorValue = maxValue;
}

/*! Get the maximum sensor value
 @return double
*/
double CssComm::getMaxSensorValue() const { return this->maxSensorValue; }

/*! Set the cheby polynomial count
 @return void
 @param count [-] cheby polynomial count
*/
void CssComm::setChebyCount(const uint32_t count) {
    if (count <= 0) {
        throw std::invalid_argument("The cheby polynomial count must be positive.");
    }
    if (count > kMaxNumChebyPolys) {
        throw std::invalid_argument("The cheby polynomial count exceeds the maximum allowed.");
    }
    this->chebyCount = count;
}

/*! Get the cheby polynomial count
 @return uint32_t
*/
uint32_t CssComm::getChebyCount() const { return this->chebyCount; }

/*! Set the cheby polynomials
 @return void
 @param polynomials [-] cheby polynomials
*/
void CssComm::setChebyPolynomials(const std::array<double, kMaxNumChebyPolys>& polynomials) {
    this->chebyPolynomials = polynomials;
}

/*! Get the cheby polynomials
 @return std::array<double, kMaxNumChebyPolys>
*/
std::array<double, kMaxNumChebyPolys> CssComm::getChebyPolynomials() const { return this->chebyPolynomials; }
