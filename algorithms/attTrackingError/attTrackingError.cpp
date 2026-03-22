#include "attTrackingError.h"

#include "../../architecture/utilities/messageConversionHelpers.hpp"
#include "architecture/utilities/eigenSupport.h"

/*! This method performs a complete reset of the module. Local module variables that retain time varying states between
 function calls are reset to their default values.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void AttTrackingError::reset(uint64_t callTime) {
    // check if the required input messages are included
    if (!this->attRefInMsg.isLinked()) {
        throw std::invalid_argument("attTrackingError.attRefInMsg wasn't connected.");
    }
    if (!this->attNavInMsg.isLinked()) {
        throw std::invalid_argument("attTrackingError.attNavInMsg wasn't connected.");
    }
}

/*! The Update method performs reads the Navigation message (containing the spacecraft attitude information), and the
 Reference message (containing the desired attitude). It computes the attitude error and writes it in the Guidance
 message.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void AttTrackingError::updateState(uint64_t callTime) {
    AttRefMsgF32Payload refF32{};
    convert(this->attRefInMsg(), refF32);
    NavAttMsgF32Payload navF32{};
    convert(this->attNavInMsg(), navF32);

    AttGuidMsgPayload attGuidOut{};
    const AttGuidMsgF32Payload attGuidOutF32 = this->algorithm.update(refF32, navF32);
    convert(attGuidOutF32, attGuidOut);

    this->attGuidOutMsg.write(&attGuidOut, this->moduleID, callTime);
}
