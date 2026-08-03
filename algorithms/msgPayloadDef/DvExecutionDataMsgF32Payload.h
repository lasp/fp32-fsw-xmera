#ifndef DV_EXECUTION_DATA_F32_MESSAGE_H
#define DV_EXECUTION_DATA_F32_MESSAGE_H

#include <stdint.h>

/*! @brief DV execution data structure */
typedef struct {
    uint32_t burnExecuting;  //!< [-] Flag indicating whether the burn is executing
    uint32_t burnComplete;   //!< [-] Flag indicating whether the burn is complete
} DvExecutionDataMsgF32Payload;

#endif
