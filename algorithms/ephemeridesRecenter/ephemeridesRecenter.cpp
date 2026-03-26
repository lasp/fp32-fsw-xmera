/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#include "ephemeridesRecenter.h"
#include <stdexcept>

/*! @brief This method resets the module.
 @return void
 @param callTime : The clock time at which the function was called (nanoseconds)
 */
void EphemeridesRecenter::reset(const uint64_t callTime) {
    for (auto i = 0; i < this->ephemeridesNumber; ++i) {
        if (!this->ephemerides[i].inputEphemerisMsg.isLinked()) {
            throw std::invalid_argument("Input ephemeris message was not connected for " +
                                        std::to_string(this->ephemerides[i].bodySpiceId));
        }
    }
    this->ephemeridesNumber = this->getNumberOfBodies();
    this->algorithm.reset();
}

/*! @brief This method recomputes the body positions and velocities relative to
    the base body ephemeris and writes out updated ephemeris position and velocity
    for each body.
 @return void
 @param callTime : The clock time at which the function was called (nanoseconds)
 */
void EphemeridesRecenter::updateState(const uint64_t callTime) {
    std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES> bodyPayloads{};
    for (auto i = 0; i < this->ephemeridesNumber; ++i) {
        BodyEphemerisPayload newBodyPayload{};
        newBodyPayload.bodySpiceId = this->ephemerides.at(i).bodySpiceId;
        newBodyPayload.originalCentralBodyId = this->ephemerides.at(i).originalCentralBodyId;
        for (auto j = 0; j < 3; j++) {
            newBodyPayload.input_r[j] = this->ephemerides[i].inputEphemerisMsg().r_BdyZero_N[j];
            newBodyPayload.input_v[j] = this->ephemerides[i].inputEphemerisMsg().v_BdyZero_N[j];
        }
        bodyPayloads[i] = newBodyPayload;
    }

    auto outputPayloads = this->algorithm.updateState(bodyPayloads);

    for (auto i = 0; i < this->ephemeridesNumber; ++i) {
        EphemerisMsgF32Payload output_i{};
        for (auto j = 0; j < 3; j++) {
            output_i.r_BdyZero_N[j] = outputPayloads[i].output_r[j];
            output_i.v_BdyZero_N[j] = outputPayloads[i].output_v[j];
        }
        const EphemerisMsgF32Payload input_i = this->ephemerides[i].inputEphemerisMsg();
        output_i.timeTag = input_i.timeTag;
        this->recenteredEphemerisOutputMsgs[i]->write(&output_i, this->moduleID, callTime);
    }
}

/*! @brief Add a body to be re-centered.
 @return void
 @param ephemerisBody BodyEphemeris : A new celestial body instance
 */
void EphemeridesRecenter::addBodyEphemerisToRecenter(const BodyEphemeris& ephemerisBody) {
    if (this->ephemeridesNumber + 1 > MAX_NUM_CHANGE_BODIES) {
        throw std::invalid_argument("Adding one body too many to the list");
    }
    this->recenteredEphemerisOutputMsgs.push_back(new Message<EphemerisMsgF32Payload>);
    this->ephemerides[this->ephemeridesNumber] = ephemerisBody;
    this->ephemeridesNumber += 1;
    this->algorithm.addBodyEphemerisToRecenter({ephemerisBody.bodySpiceId, ephemerisBody.originalCentralBodyId});
}

/*! @brief Set a new celestial body center by SPICE ID
 @return void
 @param bodySpiceId int : the new zero base
 */
void EphemeridesRecenter::setNewZeroBase(int bodySpiceId) { this->algorithm.setNewZeroBaseId(bodySpiceId); }

/*! @brief Get the new celestial body center by SPICE ID
 @return int : the new zero base
 */
int EphemeridesRecenter::getNewZeroBase() const { return this->algorithm.getNewZeroBase(); }

/*! @brief Set the previous common zero base of all the celestial bodies entered
 @param bodySpiceId int : the new zero base
 */
void EphemeridesRecenter::setPreviousCommonZeroBase(int bodySpiceId) {
    this->algorithm.setPreviousCommonZeroBase(bodySpiceId);
}

/*! @brief Get the previous common zero base of all the celestial bodies entered
 @return int : the new zero base
 */
int EphemeridesRecenter::getPreviousCommonZeroBase() const { return this->algorithm.getPreviousCommonZeroBase(); }

/*! @brief Get the number of bodies that were entered into the module
 @return size_t : the number of bodies
 */
size_t EphemeridesRecenter::getNumberOfBodies() const { return this->algorithm.getNumberOfBodies(); }

/*! @brief Get all the SPICE IDs of the bodies entered
 @return std::array<int, MAX_NUM_CHANGE_BODIES> : an array of IDs
 */
std::array<int, MAX_NUM_CHANGE_BODIES> EphemeridesRecenter::getAllIds() const { return this->algorithm.getAllIds(); }

/*! @brief Find the index of a body by its SPICE ID
 @return size_t : the index of the body
 @param bodySpiceId int : the SPICE ID to look up
 */
size_t EphemeridesRecenter::findBodyIndex(int bodySpiceId) const { return this->algorithm.findBodyIndex(bodySpiceId); }

/*! @brief Clear all the bodies from the current list
 */
void EphemeridesRecenter::clearAllBodies() {
    this->ephemerides.fill(BodyEphemeris{});
    this->ephemeridesNumber = 0;
    this->algorithm.clearAllBodies();
}
