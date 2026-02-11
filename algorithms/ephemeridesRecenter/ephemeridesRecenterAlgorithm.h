/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#ifndef F32XIMERA_EPHEM_RECENTER_ALGORITHM_H
#define F32XIMERA_EPHEM_RECENTER_ALGORITHM_H

#include "msgPayloadDef/EphemerisMsgF32Payload.h"
#include <algorithm>
#include <Eigen/Dense>
#include <array>
#include <cstddef>
#include <Eigen/Dense>

inline constexpr std::size_t MAX_NUM_CHANGE_BODIES = 20U;

inline constexpr std::size_t kBodyNameMaxLen = 256U;

struct BodyName {
   public:
    constexpr BodyName() = default;

    char* data() noexcept { return buf_.data(); }

    bool empty() const noexcept { return buf_[0] == '\0'; }

    friend bool operator==(const BodyName& a, const BodyName& b) noexcept { return a.buf_ == b.buf_; }
    friend bool operator!=(const BodyName& a, const BodyName& b) noexcept { return !(a == b); }

   private:
    std::array<char, kBodyNameMaxLen> buf_{};
};

struct MoonIndexFound {
    size_t index;
    bool found;
};

/**
 * @brief Container for input/output ephemeris messages used in recentering.
 */
class BodyEphemerisPayload {
   public:
    BodyName bodySpiceName{};            //!< SPICE name of the body
    BodyName originalCentralBodyName{};  //!< Original reference body for ephemeris data
    bool isMoon{false};                  //!< Body is moon of another body in the list
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
    size_t getBodyIndexFromName(const BodyName& celestialBodyName) const;
    void setNewZeroBaseName(const BodyName& bodyName);
    size_t findNewZeroBaseIndex(const BodyName& bodyName);
    BodyName getNewZeroBase() const;
    void setPreviousCommonZeroBase(const BodyName& bodyName);
    BodyName getPreviousCommonZeroBase() const;
    size_t getNumberOfBodies() const;
    std::array<BodyName, MAX_NUM_CHANGE_BODIES> getAllNames() const;
    void addBodyEphemerisToRecenter(const BodyName& bodyName);
    void clearAllBodies();

   private:
    MoonIndexFound findMoonOfBody(const BodyEphemerisPayload& celestialBody) const;

    BodyName newCentralBodyName{};
    std::array<BodyName, MAX_NUM_CHANGE_BODIES> bodyNames{};
    size_t celestialBodyCount{};  //!< Number of primary bodies
    size_t newCentralIndex{};
    std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES> celestialBodies{};  //!< All celestial bodies involved
    BodyEphemerisPayload previousCentralBody{};                                 //!< Previous reference body
    BodyName previousCentralBodyName{};
};

#endif
