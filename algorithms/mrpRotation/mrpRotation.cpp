#include "mrpRotation.h"
#include "utilities/freestandingInvalidArgument.h"
#include <stdexcept>

/*! @brief Validate that the required input message is linked, rebuild the algorithm's
 configuration from the adapter's stored properties (which captures whether the optional
 desiredAttInMsg is connected), and reset the embedded algorithm.
 @param callTime The clock time at which the function was called (nanoseconds).
 */
void MrpRotation::reset(const uint64_t callTime) {
    // check if the required input messages are included
    if (!this->attRefInMsg.isLinked()) {
        throw std::invalid_argument("mrpRotation.attRefInMsg wasn't connected.");
    }

    this->rebuildAlgorithmConfig();
    this->algorithm.reset();
}

/*! @brief Take the input attitude reference frame and superimpose the algorithm's MRP rotation on
 top of it, producing the output guidance reference message.
 @param callTime The clock time at which the function was called (nanoseconds).
 */
void MrpRotation::updateState(const uint64_t callTime) {
    const AttRefMsgF32Payload inputRef = this->attRefInMsg();
    AttStateMsgF32Payload attStates{};

    if (this->desiredAttInMsg.isLinked()) {
        attStates = this->desiredAttInMsg();
    }

    AttRefMsgF32Payload attRefOut = this->algorithm.update(callTime, inputRef, attStates);

    /*! - write attitude guidance reference output */
    this->attRefOutMsg.write(&attRefOut, this->moduleID, callTime);
}

/*! @brief Setter for the current MRP attitude coordinate set relative to the input reference.
 Validates via MrpRotationConfig::isValidInitialSigmaRR0; throws fsw::invalid_argument on failure.
 Stores the value on the adapter and rebuilds the algorithm's configuration.
 @param sigma [-] MRP attitude relative to the input reference.
 */
void MrpRotation::setSigmaRR0(const Eigen::Vector3f& sigma) {
    if (!MrpRotationConfig::isValidInitialSigmaRR0(sigma)) {
        FSW_THROW_INVALID_ARGUMENT("mrpRotation: sigma_RR0 must be finite");
    }
    this->sigma_RR0 = sigma;
    this->rebuildAlgorithmConfig();
}

/*! @brief Getter for the current MRP attitude coordinate set stored on the adapter.
 @return Eigen::Vector3f MRP relative to the input reference (the configured initial value).
 */
Eigen::Vector3f MrpRotation::getSigmaRR0() const { return this->sigma_RR0; }

/*! @brief Setter for the angular velocity vector relative to the input reference. Validates via
 MrpRotationConfig::isValidOmegaRR0R; throws fsw::invalid_argument on failure. Stores the value on
 the adapter and rebuilds the algorithm's configuration.
 @param omega [rad/s] angular velocity vector relative to the input reference.
 */
void MrpRotation::setOmegaRR0(const Eigen::Vector3f& omega) {
    if (!MrpRotationConfig::isValidOmegaRR0R(omega)) {
        FSW_THROW_INVALID_ARGUMENT("mrpRotation: omega_RR0_R must be finite");
    }
    this->omega_RR0_R = omega;
    this->rebuildAlgorithmConfig();
}

/*! @brief Getter for the angular velocity vector stored on the adapter.
 @return Eigen::Vector3f Angular velocity vector relative to the input reference (configured value).
 */
Eigen::Vector3f MrpRotation::getOmegaRR0() const { return this->omega_RR0_R; }

/*! @brief Build a fresh MrpRotationConfig from the adapter's stored values (sigma_RR0,
 omega_RR0_R, and desiredAttInMsg link state) and push it into the algorithm. Called from setters
 and reset() so the algorithm always sees a validated configuration that matches the adapter state.
 */
void MrpRotation::rebuildAlgorithmConfig() {
    const MrpRotationConfig cfg =
        MrpRotationConfig::create(this->sigma_RR0, this->omega_RR0_R, this->desiredAttInMsg.isLinked());
    this->algorithm.setConfig(cfg);
}
