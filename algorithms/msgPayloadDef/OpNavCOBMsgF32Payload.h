#ifndef COBOPNAV_MESSAGE_F32_H
#define COBOPNAV_MESSAGE_F32_H

#include <stdint.h>

//!@brief Center of brightness optical navigation measurement message
/*! This message is output by the center of brightness module and contains the center of brightness of the image
 * that was input, as well as the validity of the image processing process, the camera ID, and the number of pixels
 * found during the computation.
 */
typedef struct {
    uint64_t timeTag;             //!< --[ns]   Current vehicle time-tag associated with measurements
    bool valid;                   //!< --  Quality of measurement
    float centerOfBrightness[2];  //!< -- [-]   Center x, y of bright pixels
    int32_t pixelsFound;          //!< -- [-] Number of bright pixels found in the image
} OpNavCOBMsgF32Payload;

#endif
