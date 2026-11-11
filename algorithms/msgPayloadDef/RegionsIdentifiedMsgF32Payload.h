#ifndef REGIONS_IDENTIFIED_F32_H
#define REGIONS_IDENTIFIED_F32_H

#include "definitions.h"

/*! @brief Regions of interest extracted by camera. Only the first numberOfRegions entries of each array are valid. */
typedef struct {
    int numberOfRegions;                     //!< [-] number of valid regions, in [0, MAX_NUMBER_REGIONS]
    double timeTag[MAX_NUMBER_REGIONS];      //!< [sec] time tag of the image which provided the region(s)
    int cornerX[MAX_NUMBER_REGIONS];         //!< [pix] top-left corner pixel coordinate x
    int cornerY[MAX_NUMBER_REGIONS];         //!< [pix] top-left corner pixel coordinate y
    int width[MAX_NUMBER_REGIONS];           //!< [pix] region width
    int height[MAX_NUMBER_REGIONS];          //!< [pix] region height
    int numberOfPixels[MAX_NUMBER_REGIONS];  //!< [-] number of pixels in the region
} RegionsIdentifiedMsgF32Payload;

#endif
