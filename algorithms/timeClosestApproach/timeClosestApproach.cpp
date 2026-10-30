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
void TimeClosestApproach::writeMessages(const float tCA, const float sigmaTca, const uint64_t currentSimNanos) {
    /*! create and zero the output message */
    TimeClosestApproachMsgF32Payload tcaMsgBuffer{};
    tcaMsgBuffer.timeClosestApproach = tCA;
    tcaMsgBuffer.standardDeviation = sigmaTca;

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

    this->algorithm = std::make_unique<TimeClosestApproachAlgorithm>();
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

    // The filter message is double; the algorithm is float.
    Eigen::Vector3f const r_BN_N = cArrayToEigenVector3<double>(filterStatePayload.state).cast<float>();
    Eigen::Vector3f const v_BN_N = cArrayToEigenVector3<double>(filterStatePayload.state + 3).cast<float>();
    // The flyby filter covariance is 8 x 8 (position, velocity, two heading-bias angles); TCA uses the
    // position-velocity block.
    Eigen::Matrix<float, 6, 6> const filterCovariance =
        cArrayToEigenMatrix<double, 8, 8>(filterStatePayload.covar).topLeftCorner<6, 6>().cast<float>();

    TimeClosestApproachOutput out_algo = this->algorithm->update(r_BN_N, v_BN_N, filterCovariance);
    this->writeMessages(out_algo.tCA, out_algo.sigmaTca, currentSimNanos);
}
