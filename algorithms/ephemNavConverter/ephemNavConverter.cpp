#include "ephemNavConverter.h"
#include <architecture/utilities/eigenSupport.h>
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
    EphemerisMsgF32Payload ephemMsgPayload{};
    if (this->ephInMsg.isWritten()) {
        ephemMsgPayload = this->ephInMsg();
    }

    InputEphemerisData ephemInputData{};
    ephemInputData.timeTag = ephemMsgPayload.timeTag;
    ephemInputData.r_BdyZero_N = cArrayToEigenVector(ephemMsgPayload.r_BdyZero_N);
    ephemInputData.v_BdyZero_N = cArrayToEigenVector(ephemMsgPayload.v_BdyZero_N);

    // Call the algorithm update method
    const OutputNavTransData navTransOutputData = this->algorithm.update(ephemInputData);

    NavTransMsgF32Payload navTransMsgPayload{};
    navTransMsgPayload.timeTag = navTransOutputData.timeTag;
    eigenVectorToCArray(navTransOutputData.r_BN_N, navTransMsgPayload.r_BN_N);
    eigenVectorToCArray(navTransOutputData.v_BN_N, navTransMsgPayload.v_BN_N);

    this->stateOutMsg.write(&navTransMsgPayload, this->moduleID, callTime);
}
