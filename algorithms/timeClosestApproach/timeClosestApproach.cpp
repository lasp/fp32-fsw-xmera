// SPDX-License-Identifier: ISC
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "timeClosestApproach.h"

/*! Module constructor */
TimeClosestApproach::TimeClosestApproach() = default;

/*! Module destructor */
TimeClosestApproach::~TimeClosestApproach() = default;

/*! Write output messages.
* @return void
* @param tCA time Closest Approach
* @param sigmaTca standard deviation of Time closest approach
* @param currentSimNanos Current sim nano

*/
void TimeClosestApproach::writeMessages(const double tCA, const double sigmaTca, const uint64_t currentSimNanos) {
    /*! create and zero the output message */
    TimeClosestApproachMsgF32Payload tcaMsgBuffer{};
    tcaMsgBuffer.timeClosestApproach = static_cast<float>(tCA);
    tcaMsgBuffer.standardDeviation = static_cast<float>(sigmaTca);

    /*! Write the output messages */
    this->tcaOutMsg.write(tcaMsgBuffer, this->moduleID, currentSimNanos);
}

/*! @brief Validate that the required input messages are linked
 @param callTime The clock time at which the function was called (nanoseconds).
 */
void TimeClosestApproach::reset(const uint64_t callTime) {
    // check if the required input messages are included
    if (!this->filterInMsg.isLinked()) {
        throw std::invalid_argument("timeClosestApproach.filterInMsg wasn't connected.");
    }
    if (!this->navFilterMsg.isLinked()) {
        throw std::invalid_argument("timeClosestApproach.navFilterMsg wasn't connected.");
    }
}

/*! This method is the main carrier for the time of closest approach calculation
 @return void
 @param currentSimNanos The current simulation time for system
 */
void TimeClosestApproach::updateState(const uint64_t currentSimNanos) {
    auto filterStatePayload = this->filterInMsg();
    auto navFilterMsgPayload = this->navFilterMsg();

    int numberOfStates = filterStatePayload.numberOfStates;
    Eigen::Vector3f r_BN_N = cArrayToEigenVector(navFilterMsgPayload.r_BN_N).cast<float>();
    Eigen::Vector3f v_BN_N = cArrayToEigenVector(navFilterMsgPayload.v_BN_N).cast<float>();
    Eigen::MatrixXf filterCovariance = cArrayToEigenMatrixX(filterStatePayload.covar, numberOfStates, numberOfStates);

    TimeClosestApproachOutput out_algo = this->algorithm.update(r_BN_N, v_BN_N, filterCovariance);
    this->writeMessages(out_algo.tCA, out_algo.sigmaTca, currentSimNanos);
}
