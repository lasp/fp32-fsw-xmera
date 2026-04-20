/*
 ISC License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

*/

#ifndef F32XMERA_FORCE_TORQUE_THR_FORCE_MAPPING_ALGORITHM_H
#define F32XMERA_FORCE_TORQUE_THR_FORCE_MAPPING_ALGORITHM_H

#include "forceTorqueThrForceMappingTypes.h"
#include "msgPayloadDef/definitions.h"
#include <stdint.h>
#include <Eigen/Core>

/*! @brief This module maps thruster forces for arbitrary forces and torques
 */
class ForceTorqueThrForceMappingAlgorithm {
   public:
    Eigen::Vector<float, MAX_EFF_CNT> update(const Eigen::Vector3f& cmdTorque,
                                               const Eigen::Vector3f& cmdForce) const;

    void setThrusters(const ThrusterArrayConfig& thrusterConfig);
    ThrusterArrayConfig getThrusters() const;
    void setCoM_B(const Eigen::Vector3f& centerOfMass);
    Eigen::Vector3f getCoM_B() const;

   private:
    uint32_t numThrusters{};  //!< The number of thrusters available on vehicle
    Eigen::Vector3f CoM_B{};  //!< [m] Center of mass of the spacecraft
    Eigen::Matrix<float, 3, MAX_EFF_CNT> rThruster_B{
        Eigen::Matrix<float, 3, MAX_EFF_CNT>::Zero()};  //!< [m] Thruster locations in body frame
    Eigen::Matrix<float, 3, MAX_EFF_CNT> gtThruster_B{
        Eigen::Matrix<float, 3, MAX_EFF_CNT>::Zero()};  //!< Thruster force unit direction vectors
};

#endif
