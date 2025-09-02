/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#ifndef ATT_GUID_MESSAGE_F32_H
#define ATT_GUID_MESSAGE_F32_H



/*! @brief Structure used to define the output definition for attitude guidance*/
typedef struct {
    float sigma_BR[3];         //!<        Current attitude error estimate (MRPs) of B relative to R*/
    float omega_BR_B[3];       //!< [r/s]  Current body error estimate of B relateive to R in B frame compoonents */
    float omega_RN_B[3];       //!< [r/s]  Reference frame rate vector of the of R relative to N in B frame components */
    float domega_RN_B[3];      //!< [r/s2] Reference frame inertial body acceleration of R relative to N in B frame components */
}AttGuidMsgF32Payload;


#endif
