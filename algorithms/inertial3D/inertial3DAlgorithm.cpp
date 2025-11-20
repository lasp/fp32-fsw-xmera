/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#include "architecture/utilities/eigenSupport.h"
#include "inertial3D.h"

/*! This method creates a fixed attitude reference message.  The desired orientation is defined within the module.
 @return AttRefMsgF32Payload
 */
AttRefMsgF32Payload Inertial3DAlgorithm::update() const {
    AttRefMsgF32Payload attRefOut{};
    eigenVectorToCArray(this->sigma_RN, attRefOut.sigma_RN);

    return attRefOut;
}

/*! Setter method for the MRP from frame N to frame R.
 @return void
 @param sigma_RN [-] MRP from frame N to frame R
*/
void Inertial3DAlgorithm::setSigmaRN(const Eigen::Vector3f& sigmaInput_RN) { this->sigma_RN = sigmaInput_RN; }

/*! Getter method for the MRP from frame N to frame R.
 @return Eigen::Vector3f
*/
Eigen::Vector3f Inertial3DAlgorithm::getSigmaRN() const { return this->sigma_RN; }
