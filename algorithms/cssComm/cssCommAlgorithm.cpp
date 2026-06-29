#include "cssCommAlgorithm.h"
#include "utilities/fsw/chebyshevUtilities.h"

CssCommAlgorithm::CssCommAlgorithm(const CssCommConfig& config) : cfg(config) { setConfig(config); }

void CssCommAlgorithm::setConfig(const CssCommConfig& config) { this->cfg = config; }

/*! This method takes the raw sensor data from the coarse sun sensors and
 converts that information to the format used by the CSS nav.
 @return corrected CSS sensor values
 @param inputValues [-] Current measured CSS value for the constellation of CSS sensors
 */
std::array<double, kMaxNumCssSensors> CssCommAlgorithm::update(
    const std::array<double, kMaxNumCssSensors>& inputValues) const {
    std::array<double, kMaxNumCssSensors> outputValues{};

    for (uint32_t i = 0; i < this->cfg.getNumSensors(); ++i) {
        double const measuredValue = inputValues.at(i) / this->cfg.getMaxSensorValues().at(i); /* Scale Sensor Data */

        /* Calculate correction using Chebyshev polynomial */
        double const correction =
            calculateChebyValue(this->cfg.getChebyPolynomials(), this->cfg.getChebyCount(), measuredValue);

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
