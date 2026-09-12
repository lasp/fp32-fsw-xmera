#ifndef CSS_UNIT_MESSAGE_F32_H
#define CSS_UNIT_MESSAGE_F32_H

/*! @brief Structure used to contain the configuration information for each sun sensor*/
typedef struct {
    float nHat_B[3];  //!< [-] CSS unit normal expressed in structure
    float CBias;      //!< [W]  Individual calibration coefficient bias for CSS.  If all CSS have the same gain, then
                      //!< this is set to 1.0. If one CSS has a 10% stronger response for the same input, then the value
                      //!< would be 1.10
} CSSUnitConfigMsgF32Payload;

#endif
