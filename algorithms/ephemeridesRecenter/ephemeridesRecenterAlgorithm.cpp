/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */
#include "ephemeridesRecenterAlgorithm.h"
#include "utilities/freestandingInvalidArgument.h"

void EphemeridesRecenterAlgorithm::reset() { this->checkConfiguration(); }

/*! @brief Recenter the ephemerides
 @param newBodies std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES> : input bodies
 @return std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES> : re-centered bodies
 */
std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES> EphemeridesRecenterAlgorithm::updateState(
    const std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES>& newBodies) const {
    Eigen::Vector3d newCentral_input_r = newBodies.at(this->newCentralIndex).input_r;
    Eigen::Vector3d newCentral_input_v = newBodies.at(this->newCentralIndex).input_v;

    /* If the new central body is a moon, first re-center it around the common
     * central body so that every body is relative to the common center */
    if (this->newCentralIsMoon) {
        newCentral_input_r += newBodies.at(this->newCentralParentIndex).input_r;
        newCentral_input_v += newBodies.at(this->newCentralParentIndex).input_v;
    }

    std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES> recenteredBodies{};
    for (size_t i = 0U; i < this->celestialBodyCount; ++i) {
        if (this->isMoonAtIndex.at(i)) {
            continue;  // moons are re-centered along with their parent below
        }

        recenteredBodies.at(i) = BodyEphemerisPayload{};
        if (this->originalCentralBodyIds.at(i) == this->previousCentralBodyId) {
            Eigen::Vector3d const relativePosition = newBodies.at(i).input_r - newCentral_input_r;
            Eigen::Vector3d const relativeVelocity = newBodies.at(i).input_v - newCentral_input_v;

            if (this->moonIndices.at(i).found && this->bodyIds.at(i) != this->previousCentralBodyId) {
                const size_t moonIdx = this->moonIndices.at(i).index;
                recenteredBodies.at(moonIdx).bodySpiceId = newBodies.at(moonIdx).bodySpiceId;
                recenteredBodies.at(moonIdx).isMoon = true;
                recenteredBodies.at(moonIdx).originalCentralBodyId = newBodies.at(moonIdx).originalCentralBodyId;
                recenteredBodies.at(moonIdx).output_r = relativePosition + newBodies.at(moonIdx).input_r;
                recenteredBodies.at(moonIdx).output_v = relativeVelocity + newBodies.at(moonIdx).input_v;
            }
            recenteredBodies.at(i) = newBodies.at(i);
            recenteredBodies.at(i).output_r = relativePosition;
            recenteredBodies.at(i).output_v = relativeVelocity;
        }
    }
    return recenteredBodies;
}

/*! @brief Set the new zero base body by SPICE ID
 @param bodySpiceId int : the new zero base
 */
void EphemeridesRecenterAlgorithm::setNewZeroBaseId(const int bodySpiceId) { this->newCentralBodyId = bodySpiceId; }

/*! @brief Get the new celestial body center by SPICE ID
 @return int : the new zero base
 */
int EphemeridesRecenterAlgorithm::getNewZeroBase() const { return this->newCentralBodyId; }

/*! @brief Set the previous common zero base of all the celestial bodies entered
 @param bodySpiceId int : the new zero base
 */
void EphemeridesRecenterAlgorithm::setPreviousCommonZeroBase(const int bodySpiceId) {
    this->findBodyIndex(bodySpiceId);  // throws if not found
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
        FSW_THROW_INVALID_ARGUMENT("Requesting all body IDs but the current celestial body count is 0");
    }
    std::array<int, MAX_NUM_CHANGE_BODIES> ids{};
    for (size_t i = 0U; i < this->celestialBodyCount; ++i) {
        if (this->bodyIds.at(i) != 0) {
            ids.at(i) = this->bodyIds.at(i);
        }
    }
    return ids;
}

/*! @brief Add celestial body to be re-centered
 @param body BodyToRecenter : the body SPICE ID and its original central body
 */
void EphemeridesRecenterAlgorithm::addBodyEphemerisToRecenter(const BodyToRecenter& body) {
    if (this->celestialBodyCount + 1U > MAX_NUM_CHANGE_BODIES) {
        FSW_THROW_INVALID_ARGUMENT("Adding one body too many to the list");
    }

    this->bodyIds.at(this->celestialBodyCount) = body.bodySpiceId;
    this->originalCentralBodyIds.at(this->celestialBodyCount) = body.originalCentralBodyId;
    this->celestialBodyCount += 1U;
}

/*! @brief Clear all the bodies from the current list
 */
void EphemeridesRecenterAlgorithm::clearAllBodies() {
    this->bodyIds.fill(0);
    this->originalCentralBodyIds.fill(0);
    this->celestialBodyCount = 0U;
}

/*! @brief Validate configuration and pre-compute moon hierarchy data.
 *  Called from reset(). Throws on invalid configurations (orphan moon, moon-of-moon, multiple moons).
 */
void EphemeridesRecenterAlgorithm::checkConfiguration() {
    // Find and validate the new central body index
    this->newCentralIndex = this->findBodyIndex(this->newCentralBodyId);

    // Reset pre-computed arrays
    this->isMoonAtIndex.fill(false);
    this->moonIndices.fill(MoonIndexFound{});
    this->newCentralIsMoon = false;

    // Identify moons and validate topology
    for (size_t i = 0U; i < this->celestialBodyCount; ++i) {
        if (this->originalCentralBodyIds.at(i) == this->previousCentralBodyId) {
            continue;  // primary body orbiting the common center — not a moon
        }

        // Body is a moon (its originalCentralBodyId != previousCentralBodyId)
        this->isMoonAtIndex.at(i) = true;

        // Validate parent exists in the list (throws if not found)
        const size_t parentIndex = this->findBodyIndex(this->originalCentralBodyIds.at(i));

        // Validate no moon-of-moon: parent must orbit the common center
        if (this->originalCentralBodyIds.at(parentIndex) != this->previousCentralBodyId) {
            FSW_THROW_INVALID_ARGUMENT("A moon's parent is itself a moon (moon-of-moon not supported)");
        }
    }

    // Validate no multiple moons per parent
    for (size_t i = 0U; i < this->celestialBodyCount; ++i) {
        if (!this->isMoonAtIndex.at(i)) {
            continue;
        }
        const int parentId = this->originalCentralBodyIds.at(i);
        for (size_t j = i + 1U; j < this->celestialBodyCount; ++j) {
            if (this->isMoonAtIndex.at(j) && this->originalCentralBodyIds.at(j) == parentId) {
                FSW_THROW_INVALID_ARGUMENT("A parent body has multiple moons in the list");
            }
        }
    }

    // Pre-compute new central moon status
    this->newCentralIsMoon = (this->originalCentralBodyIds.at(this->newCentralIndex) != this->previousCentralBodyId);
    if (this->newCentralIsMoon) {
        this->newCentralParentIndex = this->findBodyIndex(this->originalCentralBodyIds.at(this->newCentralIndex));
    }

    // Pre-compute moon-of-body lookup: for each primary body i, find its moon (if any)
    for (size_t i = 0U; i < this->celestialBodyCount; ++i) {
        if (this->isMoonAtIndex.at(i)) {
            continue;  // only compute for primary bodies
        }
        for (size_t j = 0U; j < this->celestialBodyCount; ++j) {
            if (this->originalCentralBodyIds.at(j) == this->bodyIds.at(i)) {
                MoonIndexFound const moonStruct{.index = j, .found = true};
                this->moonIndices.at(i) = moonStruct;
                break;  // at most one moon per parent (validated above)
            }
        }
    }
}

size_t EphemeridesRecenterAlgorithm::findBodyIndex(const int bodySpiceId) const {
    for (size_t i = 0U; i < this->celestialBodyCount; ++i) {
        if (this->bodyIds.at(i) == bodySpiceId) {
            return i;
        }
    }
    FSW_THROW_INVALID_ARGUMENT("Body ID not found in configured body list");
}
