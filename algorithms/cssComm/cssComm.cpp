#include "cssComm.h"
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
    std::array<double, kMaxNumCssSensors>
        inputValues{}; /* [-] Current measured CSS value for the constellation of CSS sensors */

    // read sensor list input msg
    CSSArraySensorMsgF32Payload inMsgBuffer = this->sensorListInMsg();
    std::ranges::copy(
        std::ranges::begin(inMsgBuffer.CosValue), std::ranges::end(inMsgBuffer.CosValue), inputValues.begin());

    std::array<double, kMaxNumCssSensors> outputValues = this->algorithm.update(inputValues);

    CSSArraySensorMsgF32Payload outputBuffer{};
    std::ranges::copy(outputValues.begin(), outputValues.end(), std::ranges::begin(outputBuffer.CosValue));

    /*! - Write aggregate output into output message */
    this->cssArrayOutMsg.write(&outputBuffer, this->moduleID, callTime);
}

/*! Set the number of CSS sensors
 @return void
 @param numberOfSensors [-] number of CSS sensors
*/
void CssComm::setNumSensors(const uint32_t numberOfSensors) { this->algorithm.setNumSensors(numberOfSensors); }

/*! Get the number of CSS sensors
 @return uint32_t
*/
uint32_t CssComm::getNumSensors() const { return this->algorithm.getNumSensors(); }

/*! Set the maximum sensor value
 @return void
 @param maxValue [-] maximum sensor value
*/
void CssComm::setMaxSensorValue(const double maxValue) { this->algorithm.setMaxSensorValue(maxValue); }

/*! Get the maximum sensor value
 @return double
*/
double CssComm::getMaxSensorValue() const { return this->algorithm.getMaxSensorValue(); }

/*! Set the cheby polynomial count
 @return void
 @param count [-] cheby polynomial count
*/
void CssComm::setChebyCount(const uint32_t count) { this->algorithm.setChebyCount(count); }

/*! Get the cheby polynomial count
 @return uint32_t
*/
uint32_t CssComm::getChebyCount() const { return this->algorithm.getChebyCount(); }

/*! Set the cheby polynomials
 @return void
 @param polynomials [-] cheby polynomials
*/
void CssComm::setChebyPolynomials(const std::array<double, kMaxNumChebyPolys>& polynomials) {
    this->algorithm.setChebyPolynomials(polynomials);
}

/*! Get the cheby polynomials
 @return std::array<double, kMaxNumChebyPolys>
*/
std::array<double, kMaxNumChebyPolys> CssComm::getChebyPolynomials() const {
    return this->algorithm.getChebyPolynomials();
}
