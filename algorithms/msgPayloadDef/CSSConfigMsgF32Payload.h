#ifndef CSS_CONFIG_MESSAGE_F32_H
#define CSS_CONFIG_MESSAGE_F32_H

#include "CSSUnitConfigMsgF32Payload.h"
#include "definitions.h"

#include <stdint.h>

/*! @brief Structure used to contain the configuration information for each sun sensor*/
typedef struct {
    uint32_t nCSS;                                          //!< [-] Number of coarse sun sensors in cluster
    CSSUnitConfigMsgF32Payload cssVals[kMaxNumCssSensors];  //!< [-] constellation of CSS elements
} CSSConfigMsgF32Payload;

#endif
