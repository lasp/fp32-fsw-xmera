// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "dvGuidance.h"
#include "utilities/timeConstants.h"

#include <architecture/utilities/linearAlgebra.h>
#include <architecture/utilities/rigidBodyKinematics.h>

#include <stdexcept>

void DvGuidance::reset(const uint64_t callTime) {
    if (!this->burnDataInMsg.isLinked()) {
        throw std::invalid_argument("dvGuidance.burnDataInMsg wasn't connected.");
    }
}

/*! Builds an attitude reference whose body axis tracks the commanded delta-V direction while spinning at a constant
    rate about that direction. */
void DvGuidance::updateState(const uint64_t callTime) {
    double dcm_BubN[3][3];    // inertial -> base burn frame
    double dcm_ButN[3][3];    // inertial -> current burn frame
    double dcm_ButBub[3][3];  // base burn frame -> current burn frame
    double dvHat_N[3];        // commanded delta-V direction in the inertial frame
    double bu2_N[3];          // unnormalized 2nd basis vector of the base burn frame
    double rotPRV[3];         // principal rotation vector applied to the base burn frame
    AttRefMsgPayload attCmd = {};

    const DvBurnCmdMsgPayload localBurnData = this->burnDataInMsg();

    // base burn frame: 1st basis = dvHat_N, 2nd basis perpendicular to dvHat_N and dvRotVecUnit, 3rd from cross
    v3Normalize(localBurnData.dvInrtlCmd, dvHat_N);
    v3Copy(dvHat_N, dcm_BubN[0]);
    v3Cross(localBurnData.dvRotVecUnit, dvHat_N, bu2_N);
    v3Normalize(bu2_N, dcm_BubN[1]);
    v3Cross(dcm_BubN[0], dcm_BubN[1], dcm_BubN[2]);
    v3Normalize(dcm_BubN[2], dcm_BubN[2]);

    const double burnTime =
        static_cast<double>(static_cast<int64_t>(callTime) - static_cast<int64_t>(localBurnData.burnStartTime)) *
        kNano2Sec;

    // current burn frame is base burn frame rotated about its 3rd axis by dvRotVecMag * burnTime
    v3SetZero(rotPRV);
    rotPRV[2] = 1.0;
    v3Scale(burnTime * localBurnData.dvRotVecMag, rotPRV, rotPRV);
    PRV2C(rotPRV, dcm_ButBub);
    m33MultM33(dcm_ButBub, dcm_BubN, dcm_ButN);

    C2MRP(RECAST3X3 & dcm_ButN, attCmd.sigma_RN);
    // angular rate is dvRotVecMag along the current burn frame's 3rd axis (expressed in N)
    v3Scale(localBurnData.dvRotVecMag, dcm_ButN[2], attCmd.omega_RN_N);
    v3SetZero(attCmd.domega_RN_N);

    this->attRefOutMsg.write(&attCmd, this->moduleID, callTime);
}
