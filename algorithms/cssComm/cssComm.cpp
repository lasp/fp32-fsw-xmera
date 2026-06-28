#include "cssComm.h"
#include "utilities/xmera/xmeraLifecycleException.h"
#include <algorithm>
#include <stdexcept>

/*! This method performs a complete reset of the module. It validates the input message link and builds the
 algorithm from the configured parameters.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void CssComm::reset(uint64_t callTime) {
    // check if the required message has not been connected
    if (!this->sensorListInMsg.isLinked()) {
        throw std::invalid_argument("cssComm.sensorListInMsg wasn't connected.");
    }
    auto config =
        CssCommConfig::create(this->numSensors, this->maxSensorValues, this->chebyCount, this->chebyPolynomials);
    this->algorithm = std::make_unique<CssCommAlgorithm>(config);
}

/*! This method takes the raw sensor data from the coarse sun sensors and
 converts that information to the format used by the CSS nav.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void CssComm::updateState(uint64_t callTime) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("CssComm reset() has not been called.");
    }

    std::array<double, kMaxNumCssSensors>
        inputValues{}; /* [-] Current measured CSS value for the constellation of CSS sensors */

    // read sensor list input msg
    CSSArraySensorMsgF32Payload inMsgBuffer = this->sensorListInMsg();
    std::ranges::copy(
        std::ranges::begin(inMsgBuffer.CosValue), std::ranges::end(inMsgBuffer.CosValue), inputValues.begin());

    std::array<double, kMaxNumCssSensors> outputValues = this->algorithm->update(inputValues);

    CSSArraySensorMsgF32Payload outputBuffer{};
    std::ranges::copy(outputValues.begin(), outputValues.end(), std::ranges::begin(outputBuffer.CosValue));

    /*! - Write aggregate output into output message */
    this->cssArrayOutMsg.write(outputBuffer, this->moduleID, callTime);
}
