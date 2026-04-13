#ifndef F32XMERA_CSS_COMM_ALGORITHM_H
#define F32XMERA_CSS_COMM_ALGORITHM_H

#include "cssCommTypes.h"
#include "msgPayloadDef/definitions.h"

#include <array>
#include <cstddef>
#include <cstdint>

inline constexpr std::size_t kMaxNumChebyPolys = 10;

/*! @brief Top level structure for the CSS sensor interface system.  Contains all parameters for the
 CSS interface*/
class CssCommAlgorithm final {
   public:
    std::array<double, MAX_NUM_CSS_SENSORS> update(const std::array<double, MAX_NUM_CSS_SENSORS>& inputValues) const;

    void setNumSensors(uint32_t numberOfSensors);
    uint32_t getNumSensors() const;
    void setMaxSensorValue(double maxValue);
    double getMaxSensorValue() const;
    void setChebyCount(uint32_t count);
    uint32_t getChebyCount() const;
    void setChebyPolynomials(const std::array<double, kMaxNumChebyPolys>& polynomials);
    std::array<double, kMaxNumChebyPolys> getChebyPolynomials() const;

   private:
    uint32_t numSensors{};                                     //!< The number of sensors we are processing
    double maxSensorValue{};                                   //!< Scale factor to go from sensor values to cosine
    uint32_t chebyCount{};                                     //!< Count on the number of chebyshev polynomials we have
    std::array<double, kMaxNumChebyPolys> chebyPolynomials{};  //!< Chebyshev polynomials to fit output to cosine
};

#endif
