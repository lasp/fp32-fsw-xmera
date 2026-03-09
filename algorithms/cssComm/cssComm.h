#ifndef F32XMERA_CSS_COMM_H
#define F32XMERA_CSS_COMM_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/CSSArraySensorMsgPayload.h>
#include <array>
#include <cstddef>

inline constexpr std::size_t kMaxNumChebyPolys = 32;

/*! @brief Top level structure for the CSS sensor interface system.  Contains all parameters for the
 CSS interface*/
class CssComm : public SysModel {
   public:
    void updateState(uint64_t callTime) override;
    void reset(uint64_t callTime) override;

    void setNumSensors(uint32_t numberOfSensors);
    uint32_t getNumSensors() const;
    void setMaxSensorValue(double maxValue);
    double getMaxSensorValue() const;
    void setChebyCount(uint32_t count);
    uint32_t getChebyCount() const;
    void setChebyPolynomials(const std::array<double, kMaxNumChebyPolys>& polynomials);
    std::array<double, kMaxNumChebyPolys> getChebyPolynomials() const;

    ReadFunctor<CSSArraySensorMsgPayload> sensorListInMsg;  //!< input message that contains CSS data
    Message<CSSArraySensorMsgPayload> cssArrayOutMsg;       //!< output message of corrected CSS data

   private:
    uint32_t numSensors{};                                  //!< The number of sensors we are processing
    double maxSensorValue{};                                //!< Scale factor to go from sensor values to cosine
    uint32_t chebyCount{};                                  //!< Count on the number of chebyshev polynomials we have
    std::array<double, kMaxNumChebyPolys> kellyCheby{};   //!< Chebyshev polynomials to fit output to cosine
};

#endif
