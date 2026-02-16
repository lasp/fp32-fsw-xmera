#ifndef F32XMERA_CSS_COMM_H
#define F32XMERA_CSS_COMM_H

#define MAX_NUM_CHEBY_POLYS 32

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/CSSArraySensorMsgPayload.h>

/*! @brief Top level structure for the CSS sensor interface system.  Contains all parameters for the
 CSS interface*/
class CssComm : public SysModel {
   public:
    void updateState(uint64_t callTime) override;
    void reset(uint64_t callTime) override;

    uint32_t numSensors;                                    //!< The number of sensors we are processing
    ReadFunctor<CSSArraySensorMsgPayload> sensorListInMsg;  //!< input message that contains CSS data
    Message<CSSArraySensorMsgPayload> cssArrayOutMsg;       //!< output message of corrected CSS data

    double maxSensorValue;                   //!< Scale factor to go from sensor values to cosine
    uint32_t chebyCount;                     //!< Count on the number of chebyshev polynominals we have
    double kellyCheby[MAX_NUM_CHEBY_POLYS];  //!< Chebyshev polynominals to fit output to cosine
};

#endif
