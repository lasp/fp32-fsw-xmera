#ifndef F32XMERA_SOLAR_ARRAY_REFERENCE_ALGORITHM_H
#define F32XMERA_SOLAR_ARRAY_REFERENCE_ALGORITHM_H

#include "solarArrayReferenceTypes.h"
#include <stdint.h>

#include <Eigen/Core>

enum attitudeFrame { referenceFrame = 0, bodyFrame = 1 };

/*! @brief Pure algorithm for computing solar array rotation reference angles.
 *
 * Computes the optimal solar array rotation angle to maximize solar incidence,
 * and estimates the rotation rate via finite differences.
 */
class SolarArrayReferenceAlgorithm final {
   public:
    void reset();
    SolarArrayReferenceOutput update(const Eigen::Vector3f& sigma_BN,
                                     const Eigen::Vector3f& sigma_RN,
                                     const Eigen::Vector3f& vehSunPntBdy,
                                     float theta,
                                     uint64_t callTime);

    void setA1Hat_B(const Eigen::Vector3f& axis);
    Eigen::Vector3f getA1Hat_B() const;
    void setA2Hat_B(const Eigen::Vector3f& normal);
    Eigen::Vector3f getA2Hat_B() const;
    void setAttitudeFrame(int frame);
    int getAttitudeFrame() const;

   private:
    Eigen::Vector3f a1Hat_B{Eigen::Vector3f::Zero()};  //!< solar array drive axis in body frame coordinates
    Eigen::Vector3f a2Hat_B{Eigen::Vector3f::Zero()};  //!< solar array surface normal at zero rotation
    int attitudeFrame{};  //!< flag = 1: compute theta reference based on current attitude instead of attitude reference

    int count{};          //!< counter variable for finite differences
    uint64_t priorT{};    //!< prior call time for finite differences
    float priorThetaR{};  //!< prior output msg for finite differences
};

#endif  // F32XMERA_SOLAR_ARRAY_REFERENCE_ALGORITHM_H
