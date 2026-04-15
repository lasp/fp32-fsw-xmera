// SPDX-License-Identifier: ISC
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "triad.h"

#include <stdexcept>

#include <Eigen/Core>

#include <architecture/utilities/eigenSupport.h>
#include <architecture/utilities/rigidBodyKinematics.hpp>

static constexpr float kNormEpsilon = 1e-6F;

class XmeraLifecycleException : public std::runtime_error {
   public:
    using runtime_error::runtime_error;
};

void Triad::reset(const uint64_t callTime) {
    if (!this->attNavInMsg.isLinked()) {
        throw std::invalid_argument("triad.attNavInMsg wasn't connected.");
    }

    // check how the input body heading is provided
    if (this->bodyHeadingInMsg.isLinked()) {
        this->bodyAxisInput = BodyAxisInput::inputBodyHeadingMsg;
    } else if (this->h1Hat_B.norm() > kNormEpsilon) {
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
        if (this->hHat_N.norm() > kNormEpsilon) {
            this->inertialAxisInput = InertialAxisInput::inputInertialHeadingParameter;
        } else {
            throw std::invalid_argument(
                "neither triad.inertialHeadingInMsg nor triad.ephemerisInMsg were "
                "connected and no inertial heading hHat_N was specified.");
        }
    }

    // Phase 2: Validate config and create algorithm
    auto config = TriadConfig::create(this->a1Hat_B, this->h1Hat_B, this->hHat_N, this->celestialBodyInput);
    this->algorithm = std::make_unique<TriadAlgorithm>(config);
}

void Triad::updateState(const uint64_t callTime) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("Triad reset() has not been called.");
    }

    AttRefMsgF32Payload attRefOut = {};
    const NavAttMsgF32Payload attNavIn = this->attNavInMsg();

    // Resolve inertial heading
    Eigen::Vector3f hReqHat_N;
    if (this->inertialAxisInput == InertialAxisInput::inputInertialHeadingParameter) {
        hReqHat_N = this->hHat_N.normalized();
    } else if (this->inertialAxisInput == InertialAxisInput::inputInertialHeadingMsg) {
        const InertialHeadingMsgF32Payload inertialHeadingIn = this->inertialHeadingInMsg();
        hReqHat_N = cArrayToEigenVector(inertialHeadingIn.rHat_XN_N).normalized();
    } else if (this->inertialAxisInput == InertialAxisInput::inputEphemerisMsg) {
        const EphemerisMsgF32Payload ephemerisIn = this->ephemerisInMsg();
        const NavTransMsgF32Payload transNavIn = this->transNavInMsg();
        hReqHat_N = (cArrayToEigenVector(ephemerisIn.r_BdyZero_N) - cArrayToEigenVector(transNavIn.r_BN_N))
                        .normalized()
                        .cast<float>();
    }

    // Resolve body heading
    Eigen::Vector3f hRefHat_B;
    if (this->bodyAxisInput == BodyAxisInput::inputBodyHeadingParameter) {
        hRefHat_B = this->h1Hat_B.normalized();
    } else if (this->bodyAxisInput == BodyAxisInput::inputBodyHeadingMsg) {
        const BodyHeadingMsgF32Payload bodyHeadingIn = this->bodyHeadingInMsg();
        hRefHat_B = cArrayToEigenVector(bodyHeadingIn.rHat_XB_B).normalized();
    }

    // Compute sun direction in inertial frame
    const Eigen::Matrix3f BN = mrpToDcm(cArrayToEigenVector(attNavIn.sigma_BN));
    const Eigen::Vector3f rHat_SB_B = cArrayToEigenVector(attNavIn.vehSunPntBdy).normalized();
    const Eigen::Vector3f rHat_SB_N = BN.transpose() * rHat_SB_B;

    // Run algorithm
    const Eigen::Vector3f sigma_RN = this->algorithm->update(rHat_SB_N, hReqHat_N, hRefHat_B);

    eigenVectorToCArray(sigma_RN, attRefOut.sigma_RN);
    this->attRefOutMsg.write(&attRefOut, this->moduleID, callTime);
}
