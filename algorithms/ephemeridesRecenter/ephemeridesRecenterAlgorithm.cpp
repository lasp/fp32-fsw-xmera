#include "ephemeridesRecenterAlgorithm.h"

EphemeridesRecenterAlgorithm::EphemeridesRecenterAlgorithm(const EphemeridesRecenterConfig& config) : cfg(config) {
    setConfig(config);
}

void EphemeridesRecenterAlgorithm::setConfig(const EphemeridesRecenterConfig& config) {
    this->cfg = config;
    this->precompute();
}

/*! @brief Recenter the ephemerides
 @param newBodies std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES> : input bodies
 @return std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES> : re-centered bodies
 */
std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES> EphemeridesRecenterAlgorithm::updateState(
    const std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES>& newBodies) const {
    const int previousCentralBodyId = this->cfg.getPreviousCentralBodyId();
    const std::array<int, MAX_NUM_CHANGE_BODIES>& bodyIds = this->cfg.getBodyIds();
    const std::array<int, MAX_NUM_CHANGE_BODIES>& originalCentralBodyIds = this->cfg.getOriginalCentralBodyIds();
    const size_t bodyCount = this->cfg.getBodyCount();

    Eigen::Vector3d newCentralPosition = newBodies.at(this->newCentralIndex).position;
    Eigen::Vector3d newCentralVelocity = newBodies.at(this->newCentralIndex).velocity;

    /* If the new central body is a moon, first re-center it around the common
     * central body so that every body is relative to the common center */
    if (this->newCentralIsMoon) {
        newCentralPosition += newBodies.at(this->newCentralParentIndex).position;
        newCentralVelocity += newBodies.at(this->newCentralParentIndex).velocity;
    }

    std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES> recenteredBodies{};
    for (size_t i = 0U; i < bodyCount; ++i) {
        if (this->isMoonAtIndex.at(i)) {
            continue;  // moons are re-centered along with their parent below
        }

        recenteredBodies.at(i) = BodyEphemerisPayload{};
        if (originalCentralBodyIds.at(i) == previousCentralBodyId) {
            Eigen::Vector3d const relativePosition = newBodies.at(i).position - newCentralPosition;
            Eigen::Vector3d const relativeVelocity = newBodies.at(i).velocity - newCentralVelocity;

            if (this->moonIndices.at(i).found && bodyIds.at(i) != previousCentralBodyId) {
                const size_t moonIdx = this->moonIndices.at(i).index;
                recenteredBodies.at(moonIdx).bodySpiceId = newBodies.at(moonIdx).bodySpiceId;
                recenteredBodies.at(moonIdx).isMoon = true;
                recenteredBodies.at(moonIdx).originalCentralBodyId = newBodies.at(moonIdx).originalCentralBodyId;
                recenteredBodies.at(moonIdx).position = relativePosition + newBodies.at(moonIdx).position;
                recenteredBodies.at(moonIdx).velocity = relativeVelocity + newBodies.at(moonIdx).velocity;
            }
            recenteredBodies.at(i) = newBodies.at(i);
            recenteredBodies.at(i).position = relativePosition;
            recenteredBodies.at(i).velocity = relativeVelocity;
        }
    }
    return recenteredBodies;
}

void EphemeridesRecenterAlgorithm::precompute() {
    const std::array<int, MAX_NUM_CHANGE_BODIES>& bodyIds = this->cfg.getBodyIds();
    const std::array<int, MAX_NUM_CHANGE_BODIES>& originalCentralBodyIds = this->cfg.getOriginalCentralBodyIds();
    const size_t bodyCount = this->cfg.getBodyCount();
    const int previousCentralBodyId = this->cfg.getPreviousCentralBodyId();

    this->newCentralIndex = EphemeridesRecenterConfig::findIndex(bodyIds, bodyCount, this->cfg.getNewCentralBodyId());
    this->isMoonAtIndex.fill(false);
    this->moonIndices.fill(MoonIndexFound{});
    this->newCentralIsMoon = false;

    // A body is a moon when its original central body is not the previous common center.
    for (size_t i = 0U; i < bodyCount; ++i) {
        this->isMoonAtIndex.at(i) = (originalCentralBodyIds.at(i) != previousCentralBodyId);
    }

    this->newCentralIsMoon = (originalCentralBodyIds.at(this->newCentralIndex) != previousCentralBodyId);
    if (this->newCentralIsMoon) {
        this->newCentralParentIndex =
            EphemeridesRecenterConfig::findIndex(bodyIds, bodyCount, originalCentralBodyIds.at(this->newCentralIndex));
    }

    // For each primary body i, find its moon (if any). At most one moon per parent (validated by the config).
    for (size_t i = 0U; i < bodyCount; ++i) {
        if (this->isMoonAtIndex.at(i)) {
            continue;
        }
        for (size_t j = 0U; j < bodyCount; ++j) {
            if (originalCentralBodyIds.at(j) == bodyIds.at(i)) {
                this->moonIndices.at(i) = MoonIndexFound{.index = j, .found = true};
                break;
            }
        }
    }
}
