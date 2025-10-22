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

#ifndef F32XIMERA_SUN_SEARCH_ALGORITHM_H
#define F32XIMERA_SUN_SEARCH_ALGORITHM_H

#include <stdexcept>

#include "msgPayloadDef/AttGuidMsgF32Payload.h"
#include "msgPayloadDef/NavAttMsgF32Payload.h"
#include "msgPayloadDef/VehicleConfigMsgF32Payload.h"
#include <Eigen/Dense>

#define NUM_SLEWS 3

struct SlewProperties {
    float slewTime;       //!< [s] total time for the three-axes maneuver
    float slewAngle;      //!< [rad] total angle sweep around one axis
    float slewMaxRate;    //!< [rad/s] maximum spacecraft body rate norm
    float slewMaxTorque;  //!< [Nm] maximum torque for slew
    int slewRotAxis;       //!< [-] axes about which to perform the Sun search
};

struct KinematicProperties {
    int slewRotAxis;        //!< [-] axes about which to perform the Sun search
    float slewAngAcc;      //!< [rad/s^2] angular accelerations about each rotation axis
    float slewOmegaMax;    //!< [rad/s] highes angular rate about each rotation axis
    float slewThrustTime;  //!< [s] control time of each rotation
    float slewTotalTime;   //!< [s] total slew time of each rotation
};

struct ReferenceMotionOutput {
    Eigen::Vector3f omega_RN_B{Eigen::Vector3f::Zero()};  /*!< reference angular velocity */
    Eigen::Vector3f domega_RN_B{Eigen::Vector3f::Zero()}; /*!< reference angular acceleration */
};

class SunSearchAlgorithm {
   public:
    SunSearchAlgorithm() = default;
    ~SunSearchAlgorithm() = default;

    void reset(uint64_t currentSimNanos, VehicleConfigMsgF32Payload const& vehicleConfigIn);
    AttGuidMsgF32Payload update(uint64_t currentSimNanos, NavAttMsgF32Payload& navAttIn);
    void setSlewProperties(SlewProperties slewPropertiesInput);
    void modifySlewProperties(SlewProperties slewPropertiesInput, uint32_t index);
    SlewProperties getSlewProperties(uint32_t index) const;

   private:
    void computeKinematicProperties(uint32_t const index);
    ReferenceMotionOutput computeReferenceMotion(uint64_t const currentSimNanos, uint32_t const index);

    SlewProperties slewProperties[NUM_SLEWS];
    KinematicProperties kinematicProperties[NUM_SLEWS];
    uint32_t numberOfSlews{};             //!< [-] number of slew maneuvers set
    Eigen::Vector3f principleInertias{};  //!< [kg m^2] inertias about the three principal axes
    uint64_t resetTime;                   //!< time at which reset is called
};

#endif
