/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#ifndef F32XIMERA_EPHEM_RECENTER_ALGORITHM_H
#define F32XIMERA_EPHEM_RECENTER_ALGORITHM_H

#include <Eigen/Dense>
#include <array>
#include <cstddef>

inline constexpr std::size_t MAX_NUM_CHANGE_BODIES = 20U;

struct MoonIndexFound {
    size_t index;
    bool found;
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
        const std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES>& newBodies);
    size_t getBodyIndexFromId(int bodySpiceId) const;
    void setNewZeroBaseId(int bodySpiceId);
    size_t findNewZeroBaseIndex(int bodySpiceId);
    int getNewZeroBase() const;
    void setPreviousCommonZeroBase(int bodySpiceId);
    int getPreviousCommonZeroBase() const;
    size_t getNumberOfBodies() const;
    std::array<int, MAX_NUM_CHANGE_BODIES> getAllIds() const;
    void addBodyEphemerisToRecenter(int bodySpiceId);
    void clearAllBodies();

   private:
    MoonIndexFound findMoonOfBody(const BodyEphemerisPayload& celestialBody) const;
    static void validateNoMultipleMoons(const std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES>& bodies,
                                        size_t count);

    int newCentralBodyId{};
    std::array<int, MAX_NUM_CHANGE_BODIES> bodyIds{};
    size_t celestialBodyCount{};  //!< Number of primary bodies
    size_t newCentralIndex{};
    std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES> celestialBodies{};  //!< All celestial bodies involved
    int previousCentralBodyId{};
};

#endif
