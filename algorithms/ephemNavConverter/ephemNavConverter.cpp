/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
*/

#include "ephemNavConverter.h"

#include <stdexcept>

/*! Reset method for the module adapter interface.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void EphemNavConverter::reset(uint64_t callTime) {
    // check if the required message has not been connected
    if (!this->ephInMsg.isLinked()) {
        throw std::invalid_argument("ephemNavConverter.ephInMsg wasn't connected.");
    }
}

/*! Update method for the module adapter interface. This method also calls the algorithm update method.
 @return void
 @param callTime [ns] Time the method is called
 */
void EphemNavConverter::updateState(uint64_t callTime) {
    auto ephemMsgPayload = EphemerisMsgF32Payload();
    if (this->ephInMsg.isWritten()) {
        ephemMsgPayload = this->ephInMsg();
    }

    // Call the algorithm update method
    NavTransMsgF32Payload navTransMsgPayload = this->algorithm.update(callTime, ephemMsgPayload);

    this->stateOutMsg.write(&navTransMsgPayload, this->moduleID, callTime);
}
