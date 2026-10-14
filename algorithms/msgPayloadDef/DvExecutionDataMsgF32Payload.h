#ifndef DV_EXECUTION_DATA_F32_MESSAGE_H
#define DV_EXECUTION_DATA_F32_MESSAGE_H

#include <stdbool.h>

/*! @brief DV execution data structure */
typedef struct {
    bool burnExecuting;  //!< [-] Flag indicating whether the burn is executing
    bool burnComplete;   //!< [-] Flag indicating whether the burn is complete
} DvExecutionDataMsgF32Payload;

#endif
