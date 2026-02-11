/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#include "ephemeridesRecenter.h"

#include <algorithm>
#include <stdexcept>

static BodyName stringToBodyName(const std::string& bodyName) {
    BodyName newBodyName{};
    std::ranges::copy(bodyName.begin(), bodyName.end(), newBodyName.data());
    return newBodyName;
}

/*! @brief This method resets the module.
 @return void
 @param callTime : The clock time at which the function was called (nanoseconds)
 */
void EphemeridesRecenter::reset(const uint64_t callTime) {
    for (auto i = 0; i < this->ephemeridesNumber; ++i) {
        if (!this->ephemerides[i].inputEphemerisMsg.isLinked()) {
            throw std::invalid_argument("Input ephemeris message was not connected for " +
                                        this->ephemerides[i].bodySpiceName);
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
        newBodyPayload.bodySpiceName = stringToBodyName(this->ephemerides.at(i).bodySpiceName);
        newBodyPayload.originalCentralBodyName = stringToBodyName(this->ephemerides.at(i).originalCentralBodyName);
        for (auto j = 0; j< 3; j++) {
            newBodyPayload.input_r[j] = this->ephemerides[i].inputEphemerisMsg().r_BdyZero_N[j];
            newBodyPayload.input_v[j] = this->ephemerides[i].inputEphemerisMsg().v_BdyZero_N[j];
        }
        bodyPayloads[i] = newBodyPayload;
    }

    auto outputPayloads = this->algorithm.updateState(bodyPayloads);

    for (auto i = 0; i < this->ephemeridesNumber; ++i) {
        EphemerisMsgF32Payload output_i{};
        for (auto j = 0; j< 3; j++) {
            output_i.r_BdyZero_N[j] = outputPayloads[i].output_r[j];
            output_i.v_BdyZero_N[j] = outputPayloads[i].output_v[j];
        }
        const EphemerisMsgF32Payload input_i = this->ephemerides[i].inputEphemerisMsg();
        output_i.timeTag = input_i.timeTag;
        this->recenteredEphemerisOutputMsgs[i]->write(
            &output_i, this->moduleID, callTime);
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
    this->algorithm.addBodyEphemerisToRecenter(stringToBodyName(ephemerisBody.bodySpiceName));
}

/*! @brief Set a new celestial body center by name
 @return void
 @param bodyName std::string : the new zero base
 */
void EphemeridesRecenter::setNewZeroBase(const std::string& bodyName) {
    this->algorithm.setNewZeroBaseName(stringToBodyName(bodyName));
}

/*! @brief Get the new celestial body center by name
 @return std::string : the new zero base
 */
std::string EphemeridesRecenter::getNewZeroBase() const { return {this->algorithm.getNewZeroBase().data()}; }

/*! @brief Set the previous common zero base of all the celestial bodies entered
 @param bodyName std::string : the new zero base
 */
void EphemeridesRecenter::setPreviousCommonZeroBase(const std::string& bodyName) {
    this->algorithm.setPreviousCommonZeroBase(stringToBodyName(bodyName));
}

/*! @brief Get the previous common zero base of all the celestial bodies entered
 @return std::string : the new zero base
 */
std::string EphemeridesRecenter::getPreviousCommonZeroBase() const {
    return {this->algorithm.getPreviousCommonZeroBase().data()};
}

/*! @brief Get the number of bodies that were entered into the module
 @return size_t : the number of bodies
 */
size_t EphemeridesRecenter::getNumberOfBodies() const { return this->algorithm.getNumberOfBodies(); }

/*! @brief Get the index of a body
 @param celestialBodyName std::string : celestial body name
 @return size_t : whether or not the index was found
 */
size_t EphemeridesRecenter::getBodyIndexFromName(const std::string& celestialBodyName) const {
    return this->algorithm.getBodyIndexFromName(stringToBodyName(celestialBodyName));
}

/*! @brief Get all the names of the bodies entered
 @return std::array<std::string, MAX_NUM_CHANGE_BODIES> : an array of names
 */
std::array<std::string, MAX_NUM_CHANGE_BODIES> EphemeridesRecenter::getAllNames() const {
    auto bodyNames = this->algorithm.getAllNames();
    std::array<std::string, MAX_NUM_CHANGE_BODIES> bodyNamesAsStrings{};
    for (size_t i = 0; i < MAX_NUM_CHANGE_BODIES; ++i) {
        bodyNamesAsStrings.at(i) = std::string(bodyNames.at(i).data());
    }
    return bodyNamesAsStrings;
}

/*! @brief Clear all the bodies from the current list
 */
void EphemeridesRecenter::clearAllBodies() {
    std::ranges::fill(this->ephemerides, BodyEphemeris{});
    this->ephemeridesNumber = 0;
    this->algorithm.clearAllBodies();
}
