#ifndef F32XMERA_SOLAR_ARRAY_REFERENCE_ALGORITHM_H
#define F32XMERA_SOLAR_ARRAY_REFERENCE_ALGORITHM_H

#include <stdint.h>

#include <Eigen/Core>

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

    void setA1Hat_B(const Eigen::Vector3f& axis);
    Eigen::Vector3f getA1Hat_B() const;
    void setA2Hat_B(const Eigen::Vector3f& normal);
    Eigen::Vector3f getA2Hat_B() const;

   private:
    Eigen::Vector3f a1Hat_B{Eigen::Vector3f::Zero()};  //!< solar array drive axis in body frame coordinates
    Eigen::Vector3f a2Hat_B{Eigen::Vector3f::Zero()};  //!< solar array surface normal at zero rotation
};

#endif  // F32XMERA_SOLAR_ARRAY_REFERENCE_ALGORITHM_H
