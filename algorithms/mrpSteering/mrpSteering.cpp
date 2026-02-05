/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#include "mrpSteering.h"
#include <stdexcept>

class XmeraLifecycleException : public std::runtime_error {
public:
    using runtime_error::runtime_error;
};

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
*/
void MrpSteering::reset(const uint64_t callTime) {
    // check for required input message
    if (!this->guidInMsg.isLinked()) {
        throw std::invalid_argument("mrpSteering.guidInMsg wasn't connected.");
    }
    auto config = MrpSteeringConfig::create(this->K1, this->K3, this->omegaMax, this->ignoreOuterLoopFeedforward);
    this->algorithm = std::make_unique<MrpSteeringAlgorithm>(config);
}

/*! This method takes the attitude and rate errors relative to the Reference frame, as well as
    the reference frame angular rates and acceleration
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void MrpSteering::updateState(const uint64_t callTime) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("MrpSteering reset() has not been called.");
    }

    AttGuidMsgF32Payload guidCmd = this->guidInMsg();

    RateCmdMsgF32Payload outMsg = this->algorithm->update(guidCmd);

    this->rateCmdOutMsg.write(&outMsg, moduleID, callTime);
}
