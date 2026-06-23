// SPDX-License-Identifier: ISC
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "timeClosestApproach.h"
#include "utilities/xmera/xmeraLifecycleException.h"
#include <architecture/utilities/eigenSupport.h>

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

    auto config = TimeClosestApproachConfig::create();
    this->algorithm = std::make_unique<TimeClosestApproachAlgorithm>(config);
}

/*! This method is the main carrier for the time of closest approach calculation
 @return void
 @param currentSimNanos The current simulation time for system
 */
void TimeClosestApproach::updateState(const uint64_t currentSimNanos) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("TimeClosestApproach reset() has not been called.");
    }

    auto filterStatePayload = this->filterInMsg();

    Eigen::Vector3d r_BN_N = cArrayToEigenVector3<double>(filterStatePayload.state);
    Eigen::Vector3d v_BN_N = cArrayToEigenVector3<double>(filterStatePayload.state + 3);
    Eigen::Matrix<double, 6, 6> filterCovariance = Eigen::Matrix<double, 6, 6>::Zero();
    filterCovariance = cArrayToEigenMatrix<double, 6, 6>(filterStatePayload.covar);

    TimeClosestApproachOutput out_algo = this->algorithm->update(r_BN_N, v_BN_N, filterCovariance);
    this->writeMessages(out_algo.tCA, out_algo.sigmaTca, currentSimNanos);
}
