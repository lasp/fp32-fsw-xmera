#ifndef F32XIMERA_EPHEM_RECENTER_ALGORITHM_H
#define F32XIMERA_EPHEM_RECENTER_ALGORITHM_H

#include "ephemeridesRecenterTypes.h"
#include <Eigen/Dense>
#include <array>
#include <cstddef>

struct MoonIndexFound {
    size_t index{};
    bool found{false};
};

/**
 * @brief Container for input/output ephemeris messages used in recentering.
 */
class BodyEphemerisPayload {
   public:
    int bodySpiceId{};            //!< SPICE ID of the body
    int originalCentralBodyId{};  //!< Original reference body SPICE ID for ephemeris data
    bool isMoon{false};           //!< Body is moon of another body in the list
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
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
