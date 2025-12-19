/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
*/

#ifndef F32XIMERA_SUN_SEARCH_ALGORITHM_H
#define F32XIMERA_SUN_SEARCH_ALGORITHM_H

#include "msgPayloadDef/AttGuidMsgF32Payload.h"
#include "msgPayloadDef/NavAttMsgF32Payload.h"
#include "msgPayloadDef/VehicleConfigMsgF32Payload.h"
#include <Eigen/Core>

#define NUM_SLEWS 3

struct SlewProperties {
    float slewTime;       //!< [s] total time for the three-axes maneuver
    float slewAngle;      //!< [rad] total angle sweep around one axis
    float slewMaxRate;    //!< [rad/s] maximum spacecraft body rate norm
    float slewMaxTorque;  //!< [Nm] maximum torque for slew
    int slewRotAxis;      //!< [-] axes about which to perform the Sun search
};

struct KinematicProperties {
    int slewRotAxis;       //!< [-] axes about which to perform the Sun search
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

    void reset(uint64_t currentSimNanos, const VehicleConfigMsgF32Payload& vehicleConfigIn);
    AttGuidMsgF32Payload update(uint64_t currentSimNanos, const NavAttMsgF32Payload& navAttIn) const;
    void setSlewProperties(const SlewProperties& slewPropertiesInput);
    void modifySlewProperties(const SlewProperties& slewPropertiesInput, uint32_t index);
    SlewProperties getSlewProperties(uint32_t index) const;

   private:
    void computeKinematicProperties(uint32_t index);
    ReferenceMotionOutput computeReferenceMotion(uint64_t currentSimNanos, uint32_t index) const;

    SlewProperties slewProperties[NUM_SLEWS];
    KinematicProperties kinematicProperties[NUM_SLEWS];
    uint32_t numberOfSlews{};             //!< [-] number of slew maneuvers set
    Eigen::Vector3f principleInertias{};  //!< [kg m^2] inertias about the three principal axes
    uint64_t resetTime;                   //!< time at which reset is called
};

#endif
