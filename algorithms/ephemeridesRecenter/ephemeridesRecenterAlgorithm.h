#ifndef F32XIMERA_EPHEM_RECENTER_ALGORITHM_H
#define F32XIMERA_EPHEM_RECENTER_ALGORITHM_H

#include "ephemeridesRecenterTypes.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include <Eigen/Dense>
#include <array>
#include <cstddef>

struct MoonIndexFound {
    size_t index{};
    bool found{false};
};

/*!
 * @brief Validated configuration for the ephemerides recenter algorithm.
 *
 * Holds the new central body, the previous common central body, and the list of bodies to recenter
 * (SPICE IDs and their original central-body IDs). An instance can only exist if: the body count does not
 * exceed MAX_NUM_CHANGE_BODIES; the new central body is in the list; and the moon topology is valid (every
 * moon's parent is in the list and orbits the common center, with at most one moon per parent). Construct
 * via EphemeridesRecenterConfig::create(...).
 */
class EphemeridesRecenterConfig final {
   public:
    static EphemeridesRecenterConfig create(int newCentralBodyId,
                                            int previousCentralBodyId,
                                            const std::array<int, MAX_NUM_CHANGE_BODIES>& bodyIds,
                                            const std::array<int, MAX_NUM_CHANGE_BODIES>& originalCentralBodyIds,
                                            size_t bodyCount) {
        if (!isValidBodyCount(bodyCount)) {
            FSW_THROW_INVALID_ARGUMENT("ephemeridesRecenter: bodyCount must not exceed MAX_NUM_CHANGE_BODIES");
        }
        if (!isValidNewCentralBody(newCentralBodyId, bodyIds, bodyCount)) {
            FSW_THROW_INVALID_ARGUMENT("ephemeridesRecenter: new central body ID not found in the body list");
        }
        if (!isValidMoonTopology(previousCentralBodyId, bodyIds, originalCentralBodyIds, bodyCount)) {
            FSW_THROW_INVALID_ARGUMENT(
                "ephemeridesRecenter: invalid moon topology -- a moon's parent is missing or is itself a moon, "
                "or a parent has more than one moon in the list");
        }
        return {newCentralBodyId, previousCentralBodyId, bodyIds, originalCentralBodyIds, bodyCount};
    }

    static bool isValidBodyCount(size_t bodyCount) { return bodyCount <= MAX_NUM_CHANGE_BODIES; }

    static bool isValidNewCentralBody(int newCentralBodyId,
                                      const std::array<int, MAX_NUM_CHANGE_BODIES>& bodyIds,
                                      size_t bodyCount) {
        return findIndex(bodyIds, bodyCount, newCentralBodyId) != bodyCount;
    }

    //! A body is a moon when its original central body is not the previous common center. Valid topology requires
    //! every moon's parent to be in the list, to itself orbit the common center (no moon-of-moon), and to have at
    //! most one moon.
    static bool isValidMoonTopology(int previousCentralBodyId,
                                    const std::array<int, MAX_NUM_CHANGE_BODIES>& bodyIds,
                                    const std::array<int, MAX_NUM_CHANGE_BODIES>& originalCentralBodyIds,
                                    size_t bodyCount) {
        for (size_t i = 0U; i < bodyCount; ++i) {
            if (originalCentralBodyIds.at(i) == previousCentralBodyId) {
                continue;  // primary body orbiting the common center -- not a moon
            }
            const size_t parentIndex = findIndex(bodyIds, bodyCount, originalCentralBodyIds.at(i));
            if (parentIndex == bodyCount) {
                return false;  // orphan moon: parent not in the list
            }
            if (originalCentralBodyIds.at(parentIndex) != previousCentralBodyId) {
                return false;  // moon-of-moon: parent is itself a moon
            }
            for (size_t j = i + 1U; j < bodyCount; ++j) {
                if (originalCentralBodyIds.at(j) == originalCentralBodyIds.at(i)) {
                    return false;  // a second moon shares this parent
                }
            }
        }
        return true;
    }

    //! Returns the index of bodySpiceId in the body list, or bodyCount if it is not present.
    static size_t findIndex(const std::array<int, MAX_NUM_CHANGE_BODIES>& bodyIds, size_t bodyCount, int bodySpiceId) {
        for (size_t i = 0U; i < bodyCount; ++i) {
            if (bodyIds.at(i) == bodySpiceId) {
                return i;
            }
        }
        return bodyCount;
    }

    int getNewCentralBodyId() const { return newCentralBodyId; }
    int getPreviousCentralBodyId() const { return previousCentralBodyId; }
    const std::array<int, MAX_NUM_CHANGE_BODIES>& getBodyIds() const { return bodyIds; }
    const std::array<int, MAX_NUM_CHANGE_BODIES>& getOriginalCentralBodyIds() const { return originalCentralBodyIds; }
    size_t getBodyCount() const { return bodyCount; }

   private:
    EphemeridesRecenterConfig(int newCentralBodyId,
                              int previousCentralBodyId,
                              const std::array<int, MAX_NUM_CHANGE_BODIES>& bodyIds,
                              const std::array<int, MAX_NUM_CHANGE_BODIES>& originalCentralBodyIds,
                              size_t bodyCount)
        : newCentralBodyId(newCentralBodyId),
          previousCentralBodyId(previousCentralBodyId),
          bodyIds(bodyIds),
          originalCentralBodyIds(originalCentralBodyIds),
          bodyCount(bodyCount) {}

    int newCentralBodyId;
    int previousCentralBodyId;
    std::array<int, MAX_NUM_CHANGE_BODIES> bodyIds;
    std::array<int, MAX_NUM_CHANGE_BODIES> originalCentralBodyIds;
    size_t bodyCount;
};

/**
 * @brief Container for input/output ephemeris messages used in recentering.
 */
class BodyEphemerisPayload {
   public:
    int bodySpiceId{};            //!< SPICE ID of the body
    int originalCentralBodyId{};  //!< Original reference body SPICE ID for ephemeris data
    bool isMoon{false};           //!< Body is moon of another body in the list
    Eigen::Vector3d input_r = Eigen::Vector3d::Zero();
    Eigen::Vector3d input_v = Eigen::Vector3d::Zero();
    Eigen::Vector3d output_r = Eigen::Vector3d::Zero();
    Eigen::Vector3d output_v = Eigen::Vector3d::Zero();
};

/**
 * @brief Basilisk model to recenter ephemeris data from one central body to another.
 *
 * This class processes a set of body ephemerides and recomputes their
 * positions and velocities relative to a new central body.
 */
class EphemeridesRecenterAlgorithm {
   public:
    void reset();
    std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES> updateState(
        const std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES>& newBodies) const;
    void setNewZeroBaseId(int bodySpiceId);
    int getNewZeroBase() const;
    void setPreviousCommonZeroBase(int bodySpiceId);
    int getPreviousCommonZeroBase() const;
    size_t getNumberOfBodies() const;
    std::array<int, MAX_NUM_CHANGE_BODIES> getAllIds() const;
    void addBodyEphemerisToRecenter(const BodyToRecenter& body);
    void clearAllBodies();
    size_t findBodyIndex(int bodySpiceId) const;

   private:
    void checkConfiguration();

    int newCentralBodyId{};
    std::array<int, MAX_NUM_CHANGE_BODIES> bodyIds{};
    std::array<int, MAX_NUM_CHANGE_BODIES> originalCentralBodyIds{};
    size_t celestialBodyCount{};  //!< Number of primary bodies
    size_t newCentralIndex{};
    bool newCentralIsMoon{false};                                     //!< Whether the new central body is a moon
    size_t newCentralParentIndex{};                                   //!< Index of new central's parent
    std::array<MoonIndexFound, MAX_NUM_CHANGE_BODIES> moonIndices{};  //!< moonIndices[i] = moon of body i
    std::array<bool, MAX_NUM_CHANGE_BODIES> isMoonAtIndex{};          //!< true if body at index i is a moon
    int previousCentralBodyId{};
};

#endif
