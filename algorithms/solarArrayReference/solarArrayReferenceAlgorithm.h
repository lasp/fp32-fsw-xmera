#ifndef F32XMERA_SOLAR_ARRAY_REFERENCE_ALGORITHM_H
#define F32XMERA_SOLAR_ARRAY_REFERENCE_ALGORITHM_H

#include "solarArrayReferenceTypes.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/freestandingIsFinite.hpp"
#include <math.h>
#include <stdint.h>
#include <Eigen/Core>
#include <array>
#include <numbers>

/*! @brief Solar array drive axis and surface normal in body-frame coordinates. */
struct SolarArrayAxes {
    Eigen::Vector3f driveAxis = Eigen::Vector3f::Zero();      //!< [-] solar array drive axis in body frame
    Eigen::Vector3f surfaceNormal = Eigen::Vector3f::Zero();  //!< [-] solar array surface normal at zero rotation
};

/*!
 * @brief Validated configuration for the solar array reference algorithm.
 *
 * An instance can only exist with a drive axis and surface normal that are finite, (near-)unit and mutually
 * orthogonal; an alignment threshold in [1e-3, pi/2]; a valid tracking mode; and specified-array and offset angles
 * in [-pi, pi]. The axes are canonicalized into a right-handed orthonormal frame (a1 drive axis, a2 surface normal,
 * a3 = a1 x a2) when the configuration is built. Construct via SolarArrayReferenceConfig::create(...).
 */
class SolarArrayReferenceConfig final {
   public:
    static SolarArrayReferenceConfig create(const SolarArrayAxes& axes,
                                            float alignmentThreshold,
                                            TrackingMode trackingMode,
                                            float specifiedArrayAngle,
                                            float offsetAngle) {
        if (!isValidAxes(axes)) {
            FSW_THROW_INVALID_ARGUMENT(
                "solarArrayReference: drive axis and surface normal must be finite, unit vectors (norm within 1e-3 "
                "of 1.0) and mutually orthogonal.");
        }
        if (!isValidAlignmentThreshold(alignmentThreshold)) {
            FSW_THROW_INVALID_ARGUMENT("solarArrayReference: alignmentThreshold must be in [1e-3, pi/2].");
        }
        if (!isValidTrackingMode(trackingMode)) {
            FSW_THROW_INVALID_ARGUMENT("solarArrayReference: trackingMode must be AUTO_TRACK or SPECIFIED_ANGLE.");
        }
        if (!isValidSpecifiedArrayAngle(specifiedArrayAngle)) {
            FSW_THROW_INVALID_ARGUMENT("solarArrayReference: specifiedArrayAngle must be in [-pi, pi].");
        }
        if (!isValidOffsetAngle(offsetAngle)) {
            FSW_THROW_INVALID_ARGUMENT("solarArrayReference: offsetAngle must be in [-pi, pi].");
        }

        // Canonicalize the validated (near-)orthonormal axes into an exact right-handed orthonormal frame; the
        // inputs are validated unit and orthogonal, so this only removes rounding.
        const Eigen::Vector3f a1Hat_B = axes.driveAxis.stableNormalized();
        const Eigen::Vector3f a3Hat_B = a1Hat_B.cross(axes.surfaceNormal).stableNormalized();
        const Eigen::Vector3f a2Hat_B = a3Hat_B.cross(a1Hat_B).stableNormalized();
        return {a1Hat_B, a2Hat_B, a3Hat_B, alignmentThreshold, trackingMode, specifiedArrayAngle, offsetAngle};
    }

    static bool isValidAxes(const SolarArrayAxes& axes) {
        constexpr float normTolerance = 1e-3F;
        constexpr float maxDot = 1e-5F;
        if (!axes.driveAxis.allFinite() || !axes.surfaceNormal.allFinite()) {
            return false;
        }
        if (fabsf(axes.driveAxis.stableNorm() - 1.0F) > normTolerance ||
            fabsf(axes.surfaceNormal.stableNorm() - 1.0F) > normTolerance) {
            return false;
        }
        return fabsf(axes.driveAxis.stableNormalized().dot(axes.surfaceNormal.stableNormalized())) <= maxDot;
    }

    static bool isValidAlignmentThreshold(float alignmentThreshold) {
        constexpr float minAlignmentThreshold = 1e-3F;
        return alignmentThreshold >= minAlignmentThreshold && alignmentThreshold <= std::numbers::pi_v<float> / 2.0F;
    }

    static bool isValidTrackingMode(TrackingMode trackingMode) {
        return trackingMode == TrackingMode::AUTO_TRACK || trackingMode == TrackingMode::SPECIFIED_ANGLE;
    }

    static bool isValidSpecifiedArrayAngle(float specifiedArrayAngle) {
        return specifiedArrayAngle >= -std::numbers::pi_v<float> && specifiedArrayAngle <= std::numbers::pi_v<float>;
    }

    static bool isValidOffsetAngle(float offsetAngle) {
        return offsetAngle >= -std::numbers::pi_v<float> && offsetAngle <= std::numbers::pi_v<float>;
    }

    const Eigen::Vector3f& getDriveAxisHat_B() const { return this->a1Hat_B; }
    const Eigen::Vector3f& getSurfaceNormalHat_B() const { return this->a2Hat_B; }
    const Eigen::Vector3f& getThirdAxisHat_B() const { return this->a3Hat_B; }
    float getAlignmentThreshold() const { return this->alignmentThreshold; }
    TrackingMode getTrackingMode() const { return this->trackingMode; }
    float getSpecifiedArrayAngle() const { return this->specifiedArrayAngle; }
    float getOffsetAngle() const { return this->offsetAngle; }

   private:
    // NOLINTBEGIN(bugprone-easily-swappable-parameters): the three canonicalized axis vectors and the trailing
    // angle scalars are distinct documented quantities stored verbatim by the factory.
    SolarArrayReferenceConfig(const Eigen::Vector3f& a1Hat_B,
                              const Eigen::Vector3f& a2Hat_B,
                              const Eigen::Vector3f& a3Hat_B,
                              float alignmentThreshold,
                              TrackingMode trackingMode,
                              float specifiedArrayAngle,
                              float offsetAngle)
        : a1Hat_B(a1Hat_B),
          a2Hat_B(a2Hat_B),
          a3Hat_B(a3Hat_B),
          alignmentThreshold(alignmentThreshold),
          trackingMode(trackingMode),
          specifiedArrayAngle(specifiedArrayAngle),
          offsetAngle(offsetAngle) {}
    // NOLINTEND(bugprone-easily-swappable-parameters)

    Eigen::Vector3f a1Hat_B;    //!< [-] canonicalized solar array drive axis in body frame
    Eigen::Vector3f a2Hat_B;    //!< [-] canonicalized solar array surface normal at zero rotation
    Eigen::Vector3f a3Hat_B;    //!< [-] a1Hat_B x a2Hat_B, completes the right-handed array frame
    float alignmentThreshold;   //!< [rad] alignment threshold angle between sun direction and drive axis
    TrackingMode trackingMode;  //!< array tracking mode
    float specifiedArrayAngle;  //!< [rad] specified reference array angle when tracking mode is specified angle
    float offsetAngle;          //!< [rad] offset angle added to the determined reference angle
};

/*! @brief Pure algorithm for computing solar array rotation reference angles.
 *
 * Computes the optimal solar array rotation angle to maximize solar incidence,
 * and estimates the rotation rate via finite differences.
 */
class SolarArrayReferenceAlgorithm final {
   public:
    float update(const Eigen::Vector3f& sigma_BN,
                 const Eigen::Vector3f& sigma_RN,
                 const Eigen::Vector3f& rHatIn_SB_B,
                 float theta) const;

    void setSolarArrayAxes_B(const Eigen::Vector3f& driveAxis, const Eigen::Vector3f& surfaceNormal);
    std::array<Eigen::Vector3f, 2> getSolarArrayAxes_B() const;
    void setAlignmentThreshold(float threshold);
    float getAlignmentThreshold() const;
    void setTrackingMode(TrackingMode mode);
    TrackingMode getTrackingMode() const;
    void setSpecifiedArrayAngle(float angle);
    float getSpecifiedArrayAngle() const;
    void setOffsetAngle(float angle);
    float getOffsetAngle() const;

   private:
    Eigen::Vector3f a1Hat_B{Eigen::Vector3f::Zero()};  //!< solar array drive axis in body frame coordinates
    Eigen::Vector3f a2Hat_B{Eigen::Vector3f::Zero()};  //!< solar array surface normal at zero rotation
    float alignmentThreshold{1e-3F};  //!< [rad] alignment threshold angle between sun direction and drive axis
    TrackingMode trackingMode{TrackingMode::AUTO_TRACK};  //!< array tracking mode
    float specifiedArrayAngle{};  //!< [rad] specified reference array angle if tracking mode is specified angle
    float offsetAngle{};          //!< [rad] offset angle to be added to determined reference angle

    Eigen::Vector3f a3Hat_B{Eigen::Vector3f::Zero()};  //!< a1Hat_B x a2Hat_B, completes the right-handed array frame
};

#endif  // F32XMERA_SOLAR_ARRAY_REFERENCE_ALGORITHM_H
