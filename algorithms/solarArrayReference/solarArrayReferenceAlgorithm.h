#ifndef F32XMERA_SOLAR_ARRAY_REFERENCE_ALGORITHM_H
#define F32XMERA_SOLAR_ARRAY_REFERENCE_ALGORITHM_H

#include "solarArrayReferenceTypes.h"
#include <stdint.h>
#include <Eigen/Core>
#include <array>

/*! @brief Pure algorithm for computing solar array rotation reference angles.
 *
 * Computes the optimal solar array rotation angle to maximize solar incidence,
 * and estimates the rotation rate via finite differences.
 */
class SolarArrayReferenceAlgorithm final {
   public:
    float update(const Eigen::Vector3f& sigma_BN,
                 const Eigen::Vector3f& sigma_RN,
                 const Eigen::Vector3f& vehSunPntBdy,
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
};

#endif  // F32XMERA_SOLAR_ARRAY_REFERENCE_ALGORITHM_H
