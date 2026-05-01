// SPDX-License-Identifier: ISC
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "hillPoint.h"
#include <architecture/utilities/eigenSupport.h>
#include <architecture/utilities/rigidBodyKinematics.hpp>
#include <stdexcept>

void HillPoint::reset(uint64_t currentSimNanos) {
    if (!this->transNavInMsg.isLinked()) {
        throw std::invalid_argument("hillPoint.transNavInMsg wasn't connected.");
    }
    this->planetMsgIsLinked = this->celBodyInMsg.isLinked();
}

/*! Computes a Hill-frame attitude reference from the spacecraft's inertial position and velocity. */
void HillPoint::updateState(uint64_t currentSimNanos) {
    AttRefMsgPayload AttRefOutBuffer = AttRefMsgPayload();

    // primary planet defaults to zero pos/vel when the celBody message isn't connected
    EphemerisMsgPayload primPlanet = EphemerisMsgPayload();
    if (this->planetMsgIsLinked) {
        primPlanet = this->celBodyInMsg();
    }
    NavTransMsgPayload navData = this->transNavInMsg();

    computeHillPointingReference((Eigen::Vector3d)navData.r_BN_N,
                                 (Eigen::Vector3d)navData.v_BN_N,
                                 (Eigen::Vector3d)primPlanet.r_BdyZero_N,
                                 (Eigen::Vector3d)primPlanet.v_BdyZero_N,
                                 &AttRefOutBuffer);

    this->attRefOutMsg.write(&AttRefOutBuffer, moduleID, currentSimNanos);
}

void HillPoint::computeHillPointingReference(Eigen::Vector3d r_BN_N,
                                             Eigen::Vector3d v_BN_N,
                                             Eigen::Vector3d celBdyPositonVector,
                                             Eigen::Vector3d celBdyVelocityVector,
                                             AttRefMsgPayload* attRefOut) {
    Eigen::Vector3d relPosVector = r_BN_N - celBdyPositonVector;
    Eigen::Vector3d relVelVector = v_BN_N - celBdyVelocityVector;

    // DCM from inertial frame N to Hill reference frame R
    Eigen::Matrix3d dcm_RN;
    // first row i_r: radial unit vector
    dcm_RN.row(0) = relPosVector.normalized();
    // third row i_h: orbit angular momentum unit vector
    Eigen::Vector3d orbitAngMomentum = relPosVector.cross(relVelVector);
    dcm_RN.row(2) = orbitAngMomentum.normalized();
    // second row i_theta = i_h x i_r completes the right-handed Hill frame
    dcm_RN.row(1) = dcm_RN.row(2).cross(dcm_RN.row(0));

    Eigen::Vector3d sigma_RN = dcmToMrp(dcm_RN);
    eigenVectorToCArray(sigma_RN, attRefOut->sigma_RN);

    double orbitRadius = relPosVector.norm();

    double dfdt;    // true anomaly rate
    double ddfdt2;  // true anomaly acceleration
    if (orbitRadius > 1.0) {
        dfdt = orbitAngMomentum.norm() / (orbitRadius * orbitRadius);
        ddfdt2 = -2.0 * relVelVector.dot(dcm_RN.row(0)) / orbitRadius * dfdt;
    } else {
        // degenerate geometry (radius below threshold): zero rates rather than divide by ~0
        dfdt = 0.0;
        ddfdt2 = 0.0;
    }

    Eigen::Vector3d omega_RN_R = {0.0, 0.0, dfdt};
    Eigen::Vector3d domega_RN_R = {0.0, 0.0, ddfdt2};

    Eigen::Vector3d temp = dcm_RN.transpose() * omega_RN_R;
    eigenVectorToCArray(temp, attRefOut->omega_RN_N);
    temp = dcm_RN.transpose() * domega_RN_R;
    eigenVectorToCArray(temp, attRefOut->domega_RN_N);
}
