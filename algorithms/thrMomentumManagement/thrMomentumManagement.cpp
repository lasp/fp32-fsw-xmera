/*
    Thruster RW Momentum Management

 */

#include "thrMomentumManagement.h"
#include "utilities/fsw/eigenSupport.h"
#include <string.h>

#include <Eigen/Core>
#include <stdexcept>

/*! This method performs a complete reset of the module.  It validates that the required input messages
 are linked, caches the RW configuration, and re-arms the one-shot momentum dumping request.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void ThrMomentumManagement::reset(const uint64_t callTime) {
    // check if the required input messages are included
    if (!this->rwConfigDataInMsg.isLinked()) {
        throw std::invalid_argument("thrMomentumManagement.rwConfigDataInMsg wasn't connected.");
    }
    if (!this->rwSpeedsInMsg.isLinked()) {
        throw std::invalid_argument("thrMomentumManagement.rwSpeedsInMsg wasn't connected.");
    }

    /*! - read in the RW configuration message */
    this->rwConfigParams = this->rwConfigDataInMsg();

    /*! - reset the momentum dumping request flag */
    this->initRequest = 1;
}

/*! The RW momentum level is assessed to determine if a momentum dumping maneuver is required.
 This checking only happens once after the reset function is called.  To run this again afterwards,
 the reset function must be called again.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void ThrMomentumManagement::updateState(const uint64_t callTime) {
    CmdTorqueBodyMsgPayload controlOutMsg = {}; /* Control torque output message */
    Eigen::Vector3d hs_B;                       /* RW angular momentum */
    Eigen::Vector3d Delta_H_B;                  /* [Nms]  net desired angular momentum change */

    /*! - check if a momentum dumping check has been requested */
    if (this->initRequest == 1) {
        /*! - Read the input messages */
        const RWSpeedMsgPayload rwSpeedMsg = this->rwSpeedsInMsg(); /* Reaction wheel speed estimate message */

        /*! - compute net RW momentum magnitude */
        hs_B.setZero();
        for (int i = 0; i < this->rwConfigParams.numRW; i++) {
            hs_B += this->rwConfigParams.JsList[i] * rwSpeedMsg.wheelSpeeds[i] *
                    cArrayToEigenVector3(&this->rwConfigParams.GsMatrix_B[i * 3]);
        }
        const double hs = hs_B.norm(); /* net RW cluster angular momentum magnitude */

        /*! - check if momentum dumping is required */
        if (hs < this->hs_min) {
            /* Momentum dumping not required */
            Delta_H_B.setZero();
        } else {
            Delta_H_B = (-(hs - this->hs_min) / hs) * hs_B;
        }
        this->initRequest = 0;

        /*! - write out the output message */
        eigenVectorToCArray(Delta_H_B, controlOutMsg.torqueRequestBody);

        this->deltaHOutMsg.write(controlOutMsg, moduleID, callTime);
    }

    return;
}
