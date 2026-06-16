#include "ephemeridesRecenter.h"
#include "utilities/xmera/xmeraLifecycleException.h"
#include <stdexcept>

/*! @brief This method resets the module. It validates the input message links and builds the algorithm from the
    configured body list and central bodies.
 @return void
 @param callTime : The clock time at which the function was called (nanoseconds)
 */
void EphemeridesRecenter::reset(const uint64_t callTime) {
    for (size_t i = 0U; i < this->ephemeridesNumber; ++i) {
        if (!this->ephemerides.at(i).inputEphemerisMsg.isLinked()) {
            throw std::invalid_argument("Input ephemeris message was not connected for " +
                                        std::to_string(this->ephemerides.at(i).bodySpiceId));
        }
    }

    std::array<int, MAX_NUM_CHANGE_BODIES> bodyIds{};
    std::array<int, MAX_NUM_CHANGE_BODIES> originalCentralBodyIds{};
    for (size_t i = 0U; i < this->ephemeridesNumber; ++i) {
        bodyIds.at(i) = this->ephemerides.at(i).bodySpiceId;
        originalCentralBodyIds.at(i) = this->ephemerides.at(i).originalCentralBodyId;
    }
    auto config = EphemeridesRecenterConfig::create(
        this->newCentralBodyId, this->previousCentralBodyId, bodyIds, originalCentralBodyIds, this->ephemeridesNumber);
    this->algorithm = std::make_unique<EphemeridesRecenterAlgorithm>(config);
}

/*! @brief This method recomputes the body positions and velocities relative to
    the base body ephemeris and writes out updated ephemeris position and velocity
    for each body.
 @return void
 @param callTime : The clock time at which the function was called (nanoseconds)
 */
void EphemeridesRecenter::updateState(const uint64_t callTime) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("EphemeridesRecenter reset() has not been called.");
    }

    std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES> bodyPayloads{};
    for (size_t i = 0U; i < this->ephemeridesNumber; ++i) {
        const EphemerisMsgF32Payload input_i = this->ephemerides.at(i).inputEphemerisMsg();
        BodyEphemerisPayload newBodyPayload{};
        newBodyPayload.bodySpiceId = this->ephemerides.at(i).bodySpiceId;
        newBodyPayload.originalCentralBodyId = this->ephemerides.at(i).originalCentralBodyId;
        for (int j = 0; j < 3; ++j) {
            newBodyPayload.input_r[j] = input_i.r_BdyZero_N[j];
            newBodyPayload.input_v[j] = input_i.v_BdyZero_N[j];
        }
        bodyPayloads.at(i) = newBodyPayload;
    }

    auto outputPayloads = this->algorithm->updateState(bodyPayloads);

    for (size_t i = 0U; i < this->ephemeridesNumber; ++i) {
        EphemerisMsgF32Payload output_i{};
        for (int j = 0; j < 3; ++j) {
            output_i.r_BdyZero_N[j] = outputPayloads.at(i).output_r[j];
            output_i.v_BdyZero_N[j] = outputPayloads.at(i).output_v[j];
        }
        output_i.timeTag = this->ephemerides.at(i).inputEphemerisMsg().timeTag;
        this->recenteredEphemerisOutputMsgs.at(i)->write(output_i, this->moduleID, callTime);
    }
}

/*! @brief Add a body to be re-centered.
 @return void
 @param ephemerisBody BodyEphemeris : A new celestial body instance
 */
void EphemeridesRecenter::addBodyEphemerisToRecenter(const BodyEphemeris& ephemerisBody) {
    if (this->ephemeridesNumber + 1U > MAX_NUM_CHANGE_BODIES) {
        throw std::invalid_argument("Adding one body too many to the list");
    }
    this->recenteredEphemerisOutputMsgs.push_back(new Message<EphemerisMsgF32Payload>);
    this->ephemerides.at(this->ephemeridesNumber) = ephemerisBody;
    this->ephemeridesNumber += 1U;
}

/*! @brief Set a new celestial body center by SPICE ID
 @return void
 @param bodySpiceId int : the new zero base
 */
void EphemeridesRecenter::setNewZeroBase(int bodySpiceId) { this->newCentralBodyId = bodySpiceId; }

/*! @brief Get the new celestial body center by SPICE ID
 @return int : the new zero base
 */
int EphemeridesRecenter::getNewZeroBase() const { return this->newCentralBodyId; }

/*! @brief Set the previous common zero base of all the celestial bodies entered
 @param bodySpiceId int : the new zero base
 */
void EphemeridesRecenter::setPreviousCommonZeroBase(int bodySpiceId) { this->previousCentralBodyId = bodySpiceId; }

/*! @brief Get the previous common zero base of all the celestial bodies entered
 @return int : the new zero base
 */
int EphemeridesRecenter::getPreviousCommonZeroBase() const { return this->previousCentralBodyId; }

/*! @brief Get the number of bodies that were entered into the module
 @return size_t : the number of bodies
 */
size_t EphemeridesRecenter::getNumberOfBodies() const { return this->ephemeridesNumber; }

/*! @brief Get all the SPICE IDs of the bodies entered
 @return std::array<int, MAX_NUM_CHANGE_BODIES> : an array of IDs
 */
std::array<int, MAX_NUM_CHANGE_BODIES> EphemeridesRecenter::getAllIds() const {
    if (this->ephemeridesNumber == 0U) {
        throw std::invalid_argument("Requesting all body IDs but the current body count is 0");
    }
    std::array<int, MAX_NUM_CHANGE_BODIES> ids{};
    for (size_t i = 0U; i < this->ephemeridesNumber; ++i) {
        ids.at(i) = this->ephemerides.at(i).bodySpiceId;
    }
    return ids;
}

/*! @brief Find the index of a body by its SPICE ID
 @return size_t : the index of the body
 @param bodySpiceId int : the SPICE ID to look up
 */
size_t EphemeridesRecenter::findBodyIndex(int bodySpiceId) const {
    for (size_t i = 0U; i < this->ephemeridesNumber; ++i) {
        if (this->ephemerides.at(i).bodySpiceId == bodySpiceId) {
            return i;
        }
    }
    throw std::invalid_argument("Body ID not found in configured body list");
}

/*! @brief Clear all the bodies from the current list
 */
void EphemeridesRecenter::clearAllBodies() {
    this->ephemerides.fill(BodyEphemeris{});
    this->ephemeridesNumber = 0U;
}
