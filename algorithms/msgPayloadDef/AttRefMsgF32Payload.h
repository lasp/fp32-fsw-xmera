/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
*/

#ifndef ATT_REF_MESSAGE_F32_H
#define ATT_REF_MESSAGE_F32_H



/*! @brief Structure used to define the output definition for attitude reference generation */
typedef struct {
    float sigma_RN[3];         //!<        MRP Reference attitude of R relative to N
    float omega_RN_N[3];       //!< [r/s]  Reference frame rate vector of the of R relative to N in N frame components
    float domega_RN_N[3];      //!< [r/s2] Reference frame inertial acceleration of  R relative to N in N frame components
}AttRefMsgF32Payload;


#endif
