// SPDX-License-Identifier: ISC
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "triad.h"

#include <math.h>
#include <stdexcept>

#include <Eigen/Core>

#include <architecture/utilities/eigenMRP.h>
#include <architecture/utilities/eigenSupport.h>
#include <architecture/utilities/linearAlgebra.h>
#include <architecture/utilities/rigidBodyKinematics.hpp>

const double epsilon = 1e-12;

double SPE_angle(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2) {
    double dot = v1.dot(v2);

    double cross = v1.x() * v2.y() - v1.y() * v2.x();

    // Compute angle in radians and convert to degrees
    double angle = std::acos(std::clamp(dot / (v1.norm() * v2.norm()), -1.0, 1.0));
    angle = angle * 180.0 / M_PI;

    if (cross < 0) {
        angle = -angle;
    }

    return angle;
}

void Triad::reset(uint64_t callTime) {
    if (!this->attNavInMsg.isLinked()) {
        throw std::invalid_argument("triad.attNavInMsg wasn't connected.");
    }

    // check how the input body heading is provided
    if (this->bodyHeadingInMsg.isLinked()) {
        this->bodyAxisInput = BodyAxisInput::inputBodyHeadingMsg;
    } else if (this->h1Hat_B.norm() > epsilon) {
        this->bodyAxisInput = BodyAxisInput::inputBodyHeadingParameter;
    } else {
        throw std::invalid_argument(
            "triad.bodyHeadingInMsg wasn't connected and no body heading h1Hat_B was specified.");
    }

    // check how the input inertial heading is provided
    if (this->inertialHeadingInMsg.isLinked()) {
        this->inertialAxisInput = InertialAxisInput::inputInertialHeadingMsg;
    } else if (this->ephemerisInMsg.isLinked()) {
        if (!this->transNavInMsg.isLinked()) {
            throw std::invalid_argument("triad.ephemerisInMsg was specified but triad.transNavInMsg was not.");
        }
        this->inertialAxisInput = InertialAxisInput::inputEphemerisMsg;
    } else {
        if (this->hHat_N.norm() > epsilon) {
            this->inertialAxisInput = InertialAxisInput::inputInertialHeadingParameter;
        } else {
            throw std::invalid_argument(
                "neither triad.inertialHeadingInMsg nor triad.ephemerisInMsg were "
                "connected and no inertial heading hHat_N was specified.");
        }
    }
}

void Triad::updateState(uint64_t callTime) {
    AttRefMsgPayload attRefOut = {};

    NavAttMsgPayload attNavIn = this->attNavInMsg();

    Eigen::Vector3d hReqHat_N;
    if (this->inertialAxisInput == InertialAxisInput::inputInertialHeadingParameter) {
        hReqHat_N = this->hHat_N.normalized();
    } else if (this->inertialAxisInput == InertialAxisInput::inputInertialHeadingMsg) {
        InertialHeadingMsgPayload inertialHeadingIn = this->inertialHeadingInMsg();
        hReqHat_N = cArrayToEigenVector(inertialHeadingIn.rHat_XN_N).normalized();
    } else if (this->inertialAxisInput == InertialAxisInput::inputEphemerisMsg) {
        EphemerisMsgPayload ephemerisIn = this->ephemerisInMsg();
        NavTransMsgPayload transNavIn = this->transNavInMsg();
        hReqHat_N =
            (cArrayToEigenVector(ephemerisIn.r_BdyZero_N) - cArrayToEigenVector(transNavIn.r_BN_N)).normalized();
    }

    Eigen::Vector3d hRefHat_B;
    if (this->bodyAxisInput == BodyAxisInput::inputBodyHeadingParameter) {
        hRefHat_B = this->h1Hat_B.normalized();
    } else if (this->bodyAxisInput == BodyAxisInput::inputBodyHeadingMsg) {
        BodyHeadingMsgPayload bodyHeadingIn = this->bodyHeadingInMsg();
        hRefHat_B = cArrayToEigenVector(bodyHeadingIn.rHat_XB_B).normalized();
    }

    Eigen::MRPd sigma_BN(cArrayToEigenVector(attNavIn.sigma_BN));
    Eigen::Matrix3d BN = sigma_BN.toRotationMatrix().transpose();

    Eigen::Vector3d a1Hat_B = this->a1Hat_B.normalized();
    Eigen::Vector3d rHat_SB_B = cArrayToEigenVector(attNavIn.vehSunPntBdy).normalized();

    Eigen::Vector3d rHat_SB_N;
    rHat_SB_N = BN.transpose() * rHat_SB_B;

    if (double SPE = SPE_angle(rHat_SB_N, hReqHat_N); std::abs(SPE) < 0.5) {
        throw std::runtime_error("sun and earth reference vectors are parallel, Triad can not be used");
    }

    Eigen::Matrix3d BD;
    Eigen::Matrix3d RD;

    Eigen::Vector3d r2 = hRefHat_B;
    Eigen::Vector3d r3 = a1Hat_B.cross(hRefHat_B).normalized();

    Eigen::Vector3d r1 = r2.cross(r3);
    RD.col(0) = r1;
    RD.col(1) = r2;
    RD.col(2) = r3;

    Eigen::Vector3d n2 = hReqHat_N;
    Eigen::Vector3d n1 = rHat_SB_N.cross(hReqHat_N).normalized();
    Eigen::Vector3d n3 = n1.cross(n2);
    BD.col(0) = n1;
    BD.col(1) = n2;
    BD.col(2) = n3;

    Eigen::Matrix3d RN = RD * BD.transpose();

    Eigen::Vector3d sigma_RN = dcmToMrp(RN);

    double Sigma_RN[3];
    eigenVectorToCArray(sigma_RN, Sigma_RN);
    v3Copy(Sigma_RN, attRefOut.sigma_RN);

    this->attRefOutMsg.write(&attRefOut, this->moduleID, callTime);
}

void Triad::setA1Hat_B(const Eigen::Vector3d& a1Hat_B) { this->a1Hat_B = a1Hat_B; }
void Triad::setH1Hat_B(const Eigen::Vector3d& h1Hat_B) { this->h1Hat_B = h1Hat_B; }
void Triad::setHHat_N(const Eigen::Vector3d& hHat_N) { this->hHat_N = hHat_N; }
void Triad::setCelestialBodyInput(const CelestialBody& celestialBodyInput) {
    this->celestialBodyInput = celestialBodyInput;
}
void Triad::setBodyAxisInput(const BodyAxisInput& bodyAxisInput) { this->bodyAxisInput = bodyAxisInput; }
void Triad::setInertialAxisInput(const InertialAxisInput& inertialAxisInput) {
    this->inertialAxisInput = inertialAxisInput;
}

const Eigen::Vector3d Triad::getA1Hat_B() const { return this->a1Hat_B; }
const Eigen::Vector3d Triad::getH1Hat_B() const { return this->h1Hat_B; }
const Eigen::Vector3d Triad::getHHat_N() const { return this->hHat_N; }
const CelestialBody Triad::getCelestialBodyInput() const { return this->celestialBodyInput; }
const BodyAxisInput Triad::getBodyAxisInput() const { return this->bodyAxisInput; }
const InertialAxisInput Triad::getInertialAxisInput() const { return this->inertialAxisInput; }
