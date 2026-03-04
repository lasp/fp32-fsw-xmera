#ifndef CSS_ARRAY_SENSOR_MESSAGE_H
#define CSS_ARRAY_SENSOR_MESSAGE_H

#include "definitions.h"

/*! @brief Output structure for CSS array or constellation interface.  Each element contains the raw measurement which
 * should be a cosine value nominally */
typedef struct {
    float timeTag;                        //!< [s]   Current vehicle time-tag associated with measurements
    float CosValue[MAX_NUM_CSS_SENSORS];  //!< Current measured CSS value (ideally a cosine value) for the
                                           //!< constellation of CSS sensors
} CSSArraySensorMsgF32Payload;

#endif
