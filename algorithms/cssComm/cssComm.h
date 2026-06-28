#ifndef F32XMERA_CSS_COMM_H
#define F32XMERA_CSS_COMM_H

#include "cssCommAlgorithm.h"
#include "msgPayloadDef/CSSArraySensorMsgF32Payload.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>

#include <memory>

/*! @brief Top level structure for the CSS sensor interface system.  Contains all parameters for the
 CSS interface*/
class CssComm : public SysModel {
   public:
    CssComm() = default;
    ~CssComm() override = default;

    void updateState(uint64_t callTime) override;
    void reset(uint64_t callTime) override;

    uint32_t numSensors{};  //!< [-] number of CSS sensors to process
    std::array<double, kMaxNumCssSensors>
        maxSensorValues{};  //!< [-] per-sensor scale factor from sensor value to cosine
    uint32_t chebyCount{};  //!< [-] number of Chebyshev polynomials used for the correction fit
    std::array<double, kMaxNumChebyPolys> chebyPolynomials{};  //!< [-] Chebyshev polynomials fitting output to cosine

    ReadFunctor<CSSArraySensorMsgF32Payload> sensorListInMsg;  //!< input message that contains CSS data
    Message<CSSArraySensorMsgF32Payload> cssArrayOutMsg;       //!< output message of corrected CSS data

   private:
    std::unique_ptr<CssCommAlgorithm> algorithm = nullptr;
};

#endif
