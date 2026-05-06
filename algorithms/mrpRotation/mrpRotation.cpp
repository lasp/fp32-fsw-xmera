#include "mrpRotation.h"
#include "utilities/freestandingInvalidArgument.h"
#include <stdexcept>

/*! @brief Validate that the required input message is linked, enable the dynamic-reference path
 when the optional desiredAttInMsg is connected, and reset the embedded algorithm.
 @param callTime The clock time at which the function was called (nanoseconds).
 */
void MrpRotation::reset(const uint64_t callTime) {
    // check if the required input messages are included
    if (!this->attRefInMsg.isLinked()) {
        throw std::invalid_argument("mrpRotation.attRefInMsg wasn't connected.");
    }

    // Check if optional input messages are linked
    if (this->desiredAttInMsg.isLinked()) {
        this->algorithm.enableDynamicReference();
    }

    this->algorithm.reset();
}

/*! @brief Take the input attitude reference frame and superimpose the algorithm's MRP rotation on
 top of it, producing the output guidance reference message.
 @param callTime The clock time at which the function was called (nanoseconds).
 */
void MrpRotation::updateState(const uint64_t callTime) {
    const AttRefMsgPayload inputRef = this->attRefInMsg();
    AttStateMsgPayload attStates{};

    if (this->algorithm.isDynamicReferenceEnabled()) {
        attStates = this->desiredAttInMsg();
    }

    AttRefMsgPayload attRefOut = this->algorithm.update(callTime, inputRef, attStates);

    /*! - write attitude guidance reference output */
    this->attRefOutMsg.write(&attRefOut, this->moduleID, callTime);
}

/*! @brief Setter for the current MRP attitude coordinate set relative to the input reference.
 Throws fsw::invalid_argument when any component of sigma is not finite.
 @param sigma [-] MRP attitude relative to the input reference.
 */
void MrpRotation::setSigmaRR0(const Eigen::Vector3d& sigma) {
    if (!sigma.allFinite()) {
        FSW_THROW_INVALID_ARGUMENT("mrpRotation: sigma_RR0 must be finite");
    }
    this->algorithm.setSigmaRR0(sigma);
}

/*! @brief Getter for the current MRP attitude coordinate set relative to the input reference.
 @return const Eigen::Vector3d MRP relative to the input reference.
 */
const Eigen::Vector3d MrpRotation::getSigmaRR0() const { return this->algorithm.getSigmaRR0(); }

/*! @brief Setter for the angular velocity vector relative to the input reference. Throws
 fsw::invalid_argument when any component of omega is not finite.
 @param omega [rad/s] angular velocity vector relative to the input reference.
 */
void MrpRotation::setOmegaRR0(const Eigen::Vector3d& omega) {
    if (!omega.allFinite()) {
        FSW_THROW_INVALID_ARGUMENT("mrpRotation: omega_RR0_R must be finite");
    }
    this->algorithm.setOmegaRR0(omega);
}

/*! @brief Getter for the angular velocity vector relative to the input reference.
 @return const Eigen::Vector3d Angular velocity vector relative to the input reference.
 */
const Eigen::Vector3d MrpRotation::getOmegaRR0() const { return this->algorithm.getOmegaRR0(); }
