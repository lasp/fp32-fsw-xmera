#ifndef F32XMERA_FORCE_TORQUE_THR_FORCE_MAPPING_ALGORITHM_H
#define F32XMERA_FORCE_TORQUE_THR_FORCE_MAPPING_ALGORITHM_H

#include "forceTorqueThrForceMappingTypes.h"
#include "msgPayloadDef/definitions.h"
#include <stdint.h>
#include <Eigen/Core>
#include <array>

/*! @brief This module maps thruster forces for arbitrary forces and torques
 */
class ForceTorqueThrForceMappingAlgorithm {
   public:
    Eigen::Vector<float, MAX_EFF_CNT> update(const Eigen::Vector3f& cmdTorque_B,
                                             const Eigen::Vector3f& cmdForce_B) const;

    void computeThrusterMapping();

    void setThrusters(const ThrusterArrayConfig& thrusterConfig);
    ThrusterArrayConfig getThrusters() const;
    void setCoM_B(const Eigen::Vector3f& centerOfMass);
    Eigen::Vector3f getCoM_B() const;
    void setDesiredControlAxes(const std::array<bool, 6>& desiredControlAxes);
    std::array<bool, 6> getDesiredControlAxes() const;

   private:
    uint32_t numThrusters{};  //!< The number of thrusters available on vehicle
    Eigen::Vector3f CoM_B{};  //!< [m] Center of mass of the spacecraft
    Eigen::Matrix<float, 3, MAX_EFF_CNT> rThruster_B{
        Eigen::Matrix<float, 3, MAX_EFF_CNT>::Zero()};  //!< [m] Thruster locations in body frame
    Eigen::Matrix<float, 3, MAX_EFF_CNT> gtThruster_B{
        Eigen::Matrix<float, 3, MAX_EFF_CNT>::Zero()};  //!< Thruster force unit direction vectors
    std::array<bool, 6> desiredControlAxes_B{true, true, true, true, true, true};  //!< axes desired to be controlled

    Eigen::Matrix<float, MAX_EFF_CNT, 6> pseudoInverseDG{Eigen::Matrix<float, MAX_EFF_CNT, 6>::Zero()};
};

#endif
