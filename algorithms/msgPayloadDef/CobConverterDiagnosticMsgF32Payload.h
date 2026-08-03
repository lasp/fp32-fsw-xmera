#ifndef COB_CONVERTER_DIAGNOSTIC_MESSAGE_F32_H
#define COB_CONVERTER_DIAGNOSTIC_MESSAGE_F32_H

/*! @brief Message used to flag the outlier of cob error*/
typedef struct {
    bool coberrorOutlierTrigger;  // true if the predicted COB error >= numStandardDeviations * Standard deviations
} CobConverterDiagnosticMsgF32Payload;

#endif
