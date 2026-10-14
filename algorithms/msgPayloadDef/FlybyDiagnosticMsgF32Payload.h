#ifndef FLYBY_DIAGNOSTIC_MESSAGE_F32_H
#define FLYBY_DIAGNOSTIC_MESSAGE_F32_H

#include <stdint.h>

/*! @brief Structure used to define flyby diagnostic validity flags */
typedef struct {
    bool collinearityTrigger;             // true if vectors r and v are collinear
    bool maxRateTrigger;                  // true if the predicted rate exceeds the maximum rate of the spacecraft
    bool maxAccelerationTrigger;          // true if the predicted acceleration exceeds the maximum acceleration of the
                                          // spacecraft
    bool positionKnowledgeExceedTrigger;  // true if the position error exceeds a-priori sigma bound
    bool inputSampleRejected;          // true if this period's filter sample was unusable and left out of the average
    uint32_t rejectedSamplesInWindow;  // number of unusable samples in the averaging window ending this period, else 0
} FlybyDiagnosticMsgF32Payload;

#endif
