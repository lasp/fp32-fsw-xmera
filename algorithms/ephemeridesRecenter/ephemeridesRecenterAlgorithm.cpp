/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#include "ephemeridesRecenterAlgorithm.h"
#include "../freestandingInvalidArgument.h"
#include "architecture/utilities/eigenSupport.h"
#include <Eigen/Core>
#include <algorithm>

#include "architecture/utilities/eigenSupport.h"
#include <Eigen/Core>

void EphemeridesRecenterAlgorithm::reset() {
    this->newCentralIndex = this->findNewZeroBaseIndex(this->newCentralBodyName);
}

/*! @brief Recenter the ephemerides
 @param newBodies std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES> : input bodies
 @return std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES> : re-centered bodies
 */
std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES> EphemeridesRecenterAlgorithm::updateState(
    const std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES>& newBodies) {
    this->celestialBodies = newBodies;

    const auto newCentralBody = this->celestialBodies[this->newCentralIndex];
    Eigen::Vector3d newCentral_input_r = newCentralBody.input_r;
    Eigen::Vector3d newCentral_input_v = newCentralBody.input_v;

    /* - If the new central body is a moon (its original central body is not the common central body but another body in
     * the list) first re-center the moon around the common central body so that every body is relative to the common
     * center*/
    if (newCentralBody.originalCentralBodyName != this->previousCentralBodyName) {
        const auto moonCentralBodyIndex = this->getBodyIndexFromName(newCentralBody.originalCentralBodyName);
        Eigen::Vector3d const moonCentral_input_r = this->celestialBodies[moonCentralBodyIndex].input_r;
        Eigen::Vector3d const moonCentral_input_v = this->celestialBodies[moonCentralBodyIndex].input_v;
        newCentral_input_r = newCentral_input_r + moonCentral_input_r;
        newCentral_input_v = newCentral_input_v + moonCentral_input_v;
    }

    std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES> recenteredBodies{};
    for (size_t i = 0U; i < this->celestialBodyCount; ++i) {
        if (recenteredBodies[i].isMoon) {
            continue;
        }

        /* Moons get re-centered along with their central body and shouldn't be re-centered in this main loop */
        recenteredBodies[i] = BodyEphemerisPayload{};
        Eigen::Vector3d const newEphemerisToRecenter_input_r = newBodies[i].input_r;
        Eigen::Vector3d const newEphemerisToRecenter_input_v = newBodies[i].input_v;
        if (this->celestialBodies[i].originalCentralBodyName == this->previousCentralBodyName) {
            Eigen::Vector3d const relativePosition = newEphemerisToRecenter_input_r - newCentral_input_r;

            Eigen::Vector3d const relativeVelocity = newEphemerisToRecenter_input_v - newCentral_input_v;

            if (auto [moonIndex, moonFound] = this->findMoonOfBody(this->celestialBodies[i]);
                moonFound && this->celestialBodies[i].bodySpiceName != this->previousCentralBodyName) {
                Eigen::Vector3d const moonOfBody_input_r = this->celestialBodies[moonIndex].input_r;
                Eigen::Vector3d const moonOfBody_input_v = this->celestialBodies[moonIndex].input_v;
                Eigen::Vector3d const moonRelativePosition = relativePosition + moonOfBody_input_r;

                Eigen::Vector3d const moonRelativeVelocity = relativeVelocity + moonOfBody_input_v;

                recenteredBodies[moonIndex].bodySpiceName = this->celestialBodies[moonIndex].bodySpiceName;
                recenteredBodies[moonIndex].isMoon = true;
                recenteredBodies[moonIndex].originalCentralBodyName =
                    this->celestialBodies[moonIndex].originalCentralBodyName;
                recenteredBodies[moonIndex].output_r = moonRelativePosition;
                recenteredBodies[moonIndex].output_v = moonRelativeVelocity;
            }
            recenteredBodies[i] = newBodies[i];
            recenteredBodies[i].output_r = relativePosition;
            recenteredBodies[i].output_v = relativeVelocity;
        }
    }
    return recenteredBodies;
}

/*! @brief Find the moon index of a given body name
 @param celestialBody - celestial body name
 @return foundIndex - whether the moon name is found and the index to the body
 */
MoonIndexFound EphemeridesRecenterAlgorithm::findMoonOfBody(const BodyEphemerisPayload& celestialBody) const {
    MoonIndexFound foundIndex{};
    for (size_t i = 0U; i < this->celestialBodyCount; ++i) {
        if (this->celestialBodies[i].originalCentralBodyName == celestialBody.bodySpiceName) {
            foundIndex.index = i;
            foundIndex.found = true;
        }
    }
    return foundIndex;
}

/*! @brief Get the index of a body
 @param celestialBodyName std::string : celestial body name
 @return size_t : index
 */
size_t EphemeridesRecenterAlgorithm::getBodyIndexFromName(const BodyName& celestialBodyName) const {
    if (this->celestialBodyCount == 0U) {
        FS_THROW_INVALID_ARGUMENT("Requesting a body index but the current celestial body count is 0");
    }

    size_t foundIndex = 0U;
    bool isFound = false;
    for (size_t i = 0U; i < this->celestialBodyCount; ++i) {
        if (this->bodyNames[i] == celestialBodyName) {
            foundIndex = i;
            isFound = true;
        }
    }
    if (!isFound) {
        FS_THROW_INVALID_ARGUMENT("Requesting a body index but the body is not found");
    }
    return foundIndex;
}

/*! @brief Set the new zero base body type by name
 @param bodyName std::string : the new zero base
 */
void EphemeridesRecenterAlgorithm::setNewZeroBaseName(const BodyName& bodyName) { this->newCentralBodyName = bodyName; }

/*! @brief Find the new zero base body type by name
 @param bodyName BodyName : the new zero base
 */
size_t EphemeridesRecenterAlgorithm::findNewZeroBaseIndex(const BodyName& bodyName) {
    auto* indexOfNewZeroBase = std::ranges::find(this->bodyNames, bodyName);
    if (indexOfNewZeroBase == this->bodyNames.end()) {
        FS_THROW_INVALID_ARGUMENT("New zero base body was not in the list of existing bodies");
    }
    return static_cast<std::size_t>(std::distance(this->bodyNames.begin(), indexOfNewZeroBase));
}

/*! @brief Get the new celestial body center by name
 @return BodyName : the new zero base
 */
BodyName EphemeridesRecenterAlgorithm::getNewZeroBase() const { return this->newCentralBodyName; }

/*! @brief Set the previous common zero base of all the celestial bodies entered
 @param bodyName BodyName : the new zero base
 */
void EphemeridesRecenterAlgorithm::setPreviousCommonZeroBase(const BodyName& bodyName) {
    if (const auto* indexOfPreviousZeroBase = std::ranges::find(this->bodyNames, bodyName);
        indexOfPreviousZeroBase == this->bodyNames.end()) {
        FS_THROW_INVALID_ARGUMENT("Previous zero base body was not in the list of existing bodies");
    }

    this->previousCentralBodyName = bodyName;
}

/*! @brief Get the previous common zero base of all the celestial bodies entered
 @return BodyName : the new zero base
 */
BodyName EphemeridesRecenterAlgorithm::getPreviousCommonZeroBase() const { return this->previousCentralBodyName; }

/*! @brief Get the number of bodies that were entered into the module
 @return size_t : the number of bodies
 */
size_t EphemeridesRecenterAlgorithm::getNumberOfBodies() const { return this->celestialBodyCount; }

/*! @brief Get all the names of the bodies entered
 @return std::array<BodyName, MAX_NUM_CHANGE_BODIES> : an array of names
 */
std::array<BodyName, MAX_NUM_CHANGE_BODIES> EphemeridesRecenterAlgorithm::getAllNames() const {
    if (this->celestialBodyCount == 0U) {
        FS_THROW_INVALID_ARGUMENT("Requesting all body names but the current celestial body count is 0");
    }
    std::array<BodyName, MAX_NUM_CHANGE_BODIES> names{};
    for (size_t i = 0U; i < this->celestialBodyCount; ++i) {
        if (!this->bodyNames[i].empty()) {
            names[i] = this->bodyNames[i];
        }
    }
    return names;
}

/*! @brief Add celestial body by name
 @param bodyName BodyName : the body name to add
 */
void EphemeridesRecenterAlgorithm::addBodyEphemerisToRecenter(const BodyName& bodyName) {
    if (this->celestialBodyCount + 1U > MAX_NUM_CHANGE_BODIES) {
        FS_THROW_INVALID_ARGUMENT("Adding one body too many to the list");
    }
    if (const auto* indexInList = std::ranges::find(this->bodyNames, bodyName); indexInList != this->bodyNames.end()) {
        FS_THROW_INVALID_ARGUMENT("Body already added to list");
    }
    this->bodyNames[this->celestialBodyCount] = bodyName;
    this->celestialBodyCount += 1U;
}

/*! @brief Clear all the bodies from the current list
 */
void EphemeridesRecenterAlgorithm::clearAllBodies() {
    this->celestialBodies.fill(BodyEphemerisPayload{});
    this->bodyNames.fill(BodyName{});
    this->celestialBodyCount = 0U;
}
