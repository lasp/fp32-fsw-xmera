/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */
#include "ephemeridesRecenterAlgorithm.h"
#include "../freestandingInvalidArgument.h"
#include <algorithm>

void EphemeridesRecenterAlgorithm::reset() {
    this->newCentralIndex = this->findNewZeroBaseIndex(this->newCentralBodyId);
}

/*! @brief Recenter the ephemerides
 @param newBodies std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES> : input bodies
 @return std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES> : re-centered bodies
 */
std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES> EphemeridesRecenterAlgorithm::updateState(
    const std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES>& newBodies) {
    this->celestialBodies = newBodies;
    validateNoMultipleMoons(this->celestialBodies, this->celestialBodyCount);

    const auto newCentralBody = this->celestialBodies[this->newCentralIndex];
    Eigen::Vector3d newCentral_input_r = newCentralBody.input_r;
    Eigen::Vector3d newCentral_input_v = newCentralBody.input_v;

    /* - If the new central body is a moon (its original central body is not the common central body but another body in
     * the list) first re-center the moon around the common central body so that every body is relative to the common
     * center*/
    if (newCentralBody.originalCentralBodyId != this->previousCentralBodyId) {
        const auto moonCentralBodyIndex = this->getBodyIndexFromId(newCentralBody.originalCentralBodyId);
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
        if (this->celestialBodies[i].originalCentralBodyId == this->previousCentralBodyId) {
            Eigen::Vector3d const relativePosition = newEphemerisToRecenter_input_r - newCentral_input_r;

            Eigen::Vector3d const relativeVelocity = newEphemerisToRecenter_input_v - newCentral_input_v;

            if (auto [moonIndex, moonFound] = this->findMoonOfBody(this->celestialBodies[i]);
                moonFound && this->celestialBodies[i].bodySpiceId != this->previousCentralBodyId) {
                Eigen::Vector3d const moonOfBody_input_r = this->celestialBodies[moonIndex].input_r;
                Eigen::Vector3d const moonOfBody_input_v = this->celestialBodies[moonIndex].input_v;
                Eigen::Vector3d const moonRelativePosition = relativePosition + moonOfBody_input_r;
                Eigen::Vector3d const moonRelativeVelocity = relativeVelocity + moonOfBody_input_v;

                recenteredBodies[moonIndex].bodySpiceId = this->celestialBodies[moonIndex].bodySpiceId;
                recenteredBodies[moonIndex].isMoon = true;
                recenteredBodies[moonIndex].originalCentralBodyId =
                    this->celestialBodies[moonIndex].originalCentralBodyId;
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

/*! @brief Find the moon index of a given body
 @param celestialBody - celestial body
 @return foundIndex - whether the moon is found and the index to the body
 */
MoonIndexFound EphemeridesRecenterAlgorithm::findMoonOfBody(const BodyEphemerisPayload& celestialBody) const {
    MoonIndexFound foundIndex{};
    for (size_t i = 0U; i < this->celestialBodyCount; ++i) {
        if (this->celestialBodies[i].originalCentralBodyId == celestialBody.bodySpiceId) {
            foundIndex.index = i;
            foundIndex.found = true;
        }
    }
    return foundIndex;
}

/*! @brief Get the index of a body
 @param bodySpiceId int : celestial body SPICE ID
 @return size_t : index
 */
size_t EphemeridesRecenterAlgorithm::getBodyIndexFromId(int bodySpiceId) const {
    if (this->celestialBodyCount == 0U) {
        FS_THROW_INVALID_ARGUMENT("Requesting a body index but the current celestial body count is 0");
    }

    size_t foundIndex = 0U;
    bool isFound = false;
    for (size_t i = 0U; i < this->celestialBodyCount; ++i) {
        if (this->bodyIds[i] == bodySpiceId) {
            foundIndex = i;
            isFound = true;
        }
    }
    if (!isFound) {
        FS_THROW_INVALID_ARGUMENT("Requesting a body index but the body is not found");
    }
    return foundIndex;
}

/*! @brief Set the new zero base body by SPICE ID
 @param bodySpiceId int : the new zero base
 */
void EphemeridesRecenterAlgorithm::setNewZeroBaseId(int bodySpiceId) { this->newCentralBodyId = bodySpiceId; }

/*! @brief Find the new zero base body by SPICE ID
 @param bodySpiceId int : the new zero base
 */
size_t EphemeridesRecenterAlgorithm::findNewZeroBaseIndex(int bodySpiceId) {
    auto* indexOfNewZeroBase = std::ranges::find(this->bodyIds, bodySpiceId);
    if (indexOfNewZeroBase == this->bodyIds.end()) {
        FS_THROW_INVALID_ARGUMENT("New zero base body was not in the list of existing bodies");
    }
    return static_cast<std::size_t>(std::distance(this->bodyIds.begin(), indexOfNewZeroBase));
}

/*! @brief Get the new celestial body center by SPICE ID
 @return int : the new zero base
 */
int EphemeridesRecenterAlgorithm::getNewZeroBase() const { return this->newCentralBodyId; }

/*! @brief Set the previous common zero base of all the celestial bodies entered
 @param bodySpiceId int : the new zero base
 */
void EphemeridesRecenterAlgorithm::setPreviousCommonZeroBase(int bodySpiceId) {
    if (const auto* indexOfPreviousZeroBase = std::ranges::find(this->bodyIds, bodySpiceId);
        indexOfPreviousZeroBase == this->bodyIds.end()) {
        FS_THROW_INVALID_ARGUMENT("Previous zero base body was not in the list of existing bodies");
    }

    this->previousCentralBodyId = bodySpiceId;
}

/*! @brief Get the previous common zero base of all the celestial bodies entered
 @return int : the new zero base
 */
int EphemeridesRecenterAlgorithm::getPreviousCommonZeroBase() const { return this->previousCentralBodyId; }

/*! @brief Get the number of bodies that were entered into the module
 @return size_t : the number of bodies
 */
size_t EphemeridesRecenterAlgorithm::getNumberOfBodies() const { return this->celestialBodyCount; }

/*! @brief Get all the SPICE IDs of the bodies entered
 @return std::array<int, MAX_NUM_CHANGE_BODIES> : an array of IDs
 */
std::array<int, MAX_NUM_CHANGE_BODIES> EphemeridesRecenterAlgorithm::getAllIds() const {
    if (this->celestialBodyCount == 0U) {
        FS_THROW_INVALID_ARGUMENT("Requesting all body IDs but the current celestial body count is 0");
    }
    std::array<int, MAX_NUM_CHANGE_BODIES> ids{};
    for (size_t i = 0U; i < this->celestialBodyCount; ++i) {
        if (this->bodyIds[i] != 0) {
            ids[i] = this->bodyIds[i];
        }
    }
    return ids;
}

/*! @brief Add celestial body by SPICE ID
 @param bodySpiceId int : the body SPICE ID to add
 */
void EphemeridesRecenterAlgorithm::addBodyEphemerisToRecenter(int bodySpiceId) {
    if (this->celestialBodyCount + 1U > MAX_NUM_CHANGE_BODIES) {
        FS_THROW_INVALID_ARGUMENT("Adding one body too many to the list");
    }
    if (const auto* indexInList = std::ranges::find(this->bodyIds, bodySpiceId); indexInList != this->bodyIds.end()) {
        FS_THROW_INVALID_ARGUMENT("Body already added to list");
    }
    this->bodyIds[this->celestialBodyCount] = bodySpiceId;
    this->celestialBodyCount += 1U;
}

/*! @brief Clear all the bodies from the current list
 */
void EphemeridesRecenterAlgorithm::clearAllBodies() {
    this->celestialBodies.fill(BodyEphemerisPayload{});
    this->bodyIds.fill(0);
    this->celestialBodyCount = 0U;
}

/*! @brief Check if any parent body (non-Moon body) has multiple moons
 */
void EphemeridesRecenterAlgorithm::validateNoMultipleMoons(
    const std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES>& bodies,
    size_t count) {
    for (size_t i = 0U; i < count; ++i) {
        if (!bodies[i].isMoon) {
            continue;
        }

        const int parent = bodies[i].originalCentralBodyId;
        size_t moonCountForParent = 0U;

        for (size_t j = 0U; j < count; ++j) {
            if (bodies[j].isMoon && bodies[j].originalCentralBodyId == parent) {
                ++moonCountForParent;
                if (moonCountForParent > 1U) {
                    FS_THROW_INVALID_ARGUMENT("A parent body has multiple moons in the list");
                }
            }
        }
    }
}
