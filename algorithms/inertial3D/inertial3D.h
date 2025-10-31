/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#ifndef F32XIMERA_INERTIAL3D_H
#define F32XIMERA_INERTIAL3D_H

#include "inertial3DAlgorithm.h"
#include "msgPayloadDef/AttRefMsgF32Payload.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <stdint.h>
#include <Eigen/Core>

/*!@brief Data structure for module to compute the Inertial-3D pointing navigation solution.
 */
class Inertial3D : public SysModel {
   public:
    Inertial3D() = default;
    ~Inertial3D() final = default;

    void updateState(uint64_t callTime) override;
    void setSigmaR0N(const Eigen::Vector3f& sigma_RN);
    Eigen::Vector3f getSigmaR0N() const;

    Message<AttRefMsgF32Payload> attRefOutMsg;  //!< reference attitude output message

   private:
    Inertial3DAlgorithm algorithm{};
};

#endif
