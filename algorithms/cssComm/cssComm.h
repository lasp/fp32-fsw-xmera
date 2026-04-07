#ifndef F32XMERA_CSS_COMM_H
#define F32XMERA_CSS_COMM_H

#include "cssCommAlgorithm.h"
#include "msgPayloadDef/CSSArraySensorMsgF32Payload.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>

/*! @brief Top level structure for the CSS sensor interface system.  Contains all parameters for the
 CSS interface*/
class CssComm : public SysModel {
   public:
    CssComm() = default;
    ~CssComm() override = default;

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

    ReadFunctor<CSSArraySensorMsgF32Payload> sensorListInMsg;  //!< input message that contains CSS data
    Message<CSSArraySensorMsgF32Payload> cssArrayOutMsg;       //!< output message of corrected CSS data

   private:
    CssCommAlgorithm algorithm{};
};

#endif
