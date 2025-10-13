/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#ifndef F32XIMERA_INERTIAL3D_ALGORITHM_H
#define F32XIMERA_INERTIAL3D_ALGORITHM_H

#include "msgPayloadDef/AttRefMsgF32Payload.h"
#include <Eigen/Core>

/*!@brief Data structure for module to compute the Inertial-3D pointing navigation solution.
 */
class Inertial3DAlgorithm {
   public:
    AttRefMsgF32Payload update() const;
    void setSigmaR0N(const Eigen::Vector3f& sigma_RN);
    Eigen::Vector3f getSigmaR0N() const;

   private:
    Eigen::Vector3f sigma_R0N{Eigen::Vector3f::Zero()};  //!<  MRP from inertial frame N to corrected reference frame R
};

#endif
