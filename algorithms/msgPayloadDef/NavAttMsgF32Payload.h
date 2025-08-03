/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#ifndef NAV_ATT_MESSAGE_F32_H
#define NAV_ATT_MESSAGE_F32_H

/*! @brief Structure used to define the output definition for attitude guidance*/
typedef struct {
    float timeTag;          //!< [s]   Current vehicle time-tag associated with measurements*/
    float sigma_BN[3];      //!<       Current spacecraft attitude (MRPs) of body relative to inertial */
    float omega_BN_B[3];    //!< [r/s] Current spacecraft angular velocity vector of body frame B relative to inertial frame N, in B frame components
    float vehSunPntBdy[3];  //!<       Current sun pointing vector in body frame
}NavAttMsgF32Payload;


#endif
