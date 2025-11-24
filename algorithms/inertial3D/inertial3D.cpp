/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#include "inertial3D.h"
#include "msgPayloadDef/AttRefMsgF32Payload.h"
#include <Eigen/Core>
#include <cstdint>

/*! This method creates a fixed attitude reference message.  The desired orientation is
    defined within the module.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Inertial3D::updateState(const uint64_t callTime) {
    AttRefMsgF32Payload attRefOut = algorithm.update();

    this->attRefOutMsg.write(&attRefOut, this->moduleID, callTime);
}

/*! Setter method for the MRP from frame N to frame R.
 @return void
 @param sigma_RN [-] MRP from frame N to frame R
*/
void Inertial3D::setSigmaRN(const Eigen::Vector3f& sigma_RN) { this->algorithm.setSigmaRN(sigma_RN); }

/*! Getter method for the MRP from frame N to frame R.
 @return Eigen::Vector3f
*/
Eigen::Vector3f Inertial3D::getSigmaRN() const { return this->algorithm.getSigmaRN(); }
