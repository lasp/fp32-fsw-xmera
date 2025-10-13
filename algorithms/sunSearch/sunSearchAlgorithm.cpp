/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
*/

#include "sunSearchAlgorithm.h"
#include "architecture/utilities/eigenSupport.h"
#include <architecture/utilities/macroDefinitions.h>
#include <cmath>

/*! This method is used to reset the module.
 @return void
 @param currentSimNanos The current simulation time for system
 @param vehicleConfigIn Vehicle configuration message
 */
void SunSearchAlgorithm::reset(const uint64_t currentSimNanos, const VehicleConfigMsgF32Payload& vehicleConfigIn) {
    if (this->numberOfSlews != NUM_SLEWS) {
        throw std::invalid_argument("The number of specified slew maneuvers must be equal to 3");
    }

    this->principleInertias[0] = vehicleConfigIn.ISCPntB_B[0];
    this->principleInertias[1] = vehicleConfigIn.ISCPntB_B[4];
    this->principleInertias[2] = vehicleConfigIn.ISCPntB_B[8];

    for (uint32_t index = 0; index < NUM_SLEWS; index++) {
        this->computeKinematicProperties(index);
    }

    this->resetTime = currentSimNanos;
}

/*! This method is the main carrier for the computation of the guidance message
 @return AttGuidMsgF32Payload
 @param currentSimNanos The current simulation time for system
 @param navAttIn Navigation attitude message
 */
AttGuidMsgF32Payload SunSearchAlgorithm::update(const uint64_t currentSimNanos,
												const NavAttMsgF32Payload& navAttIn) const {
    AttGuidMsgF32Payload attGuidOut{};
    ReferenceMotionOutput referenceMotion{};

    const float CurrentSimSeconds = (currentSimNanos - this->resetTime) * NANO2SEC;

    float timeInf = 0;
    float timeSup = this->kinematicProperties[0].slewTotalTime;
    for (uint32_t index = 0; index < NUM_SLEWS; ++index) {
        if (CurrentSimSeconds >= timeInf && CurrentSimSeconds < timeSup) {
            referenceMotion = this->computeReferenceMotion(currentSimNanos, index);
            break;
        } else if (CurrentSimSeconds >= timeSup && index != NUM_SLEWS - 1) {
            timeInf += this->kinematicProperties[index].slewTotalTime;
            timeSup += this->kinematicProperties[index + 1].slewTotalTime;
        }
    }

    const Eigen::Vector3f omega_BR_B = Eigen::Map<const Eigen::Vector3f>(navAttIn.omega_BN_B) - referenceMotion.omega_RN_B;

    eigenVectorToCArray(referenceMotion.omega_RN_B, attGuidOut.omega_RN_B);
    eigenVectorToCArray(omega_BR_B, attGuidOut.omega_BR_B);
    eigenVectorToCArray(referenceMotion.domega_RN_B, attGuidOut.domega_RN_B);

    return attGuidOut;
}

/*! Define this method to compute the kinematic properties of each slew
    @return void
    */
void SunSearchAlgorithm::computeKinematicProperties(const uint32_t index) {
    const SlewProperties* SP = &this->slewProperties[index];
    const uint32_t axis = SP->slewRotAxis - 1;
    const float maxAcc = SP->slewMaxTorque / this->principleInertias[axis];

    /*! Computing fastest bang-bang slew with no coasting arc */
    float alpha = 4 * SP->slewAngle / (SP->slewTime * SP->slewTime);
    float omegaMax = 2 * SP->slewAngle / SP->slewTime;
    float totalTime = SP->slewTime;
    float thrustTime = totalTime / 2;

    /*! If angular acceleration exceeds limit, decrease acceleration and increase slew time */
    if (alpha > maxAcc) {
        alpha = maxAcc;
        totalTime = 2 * sqrt(SP->slewAngle / alpha);
        thrustTime = totalTime / 2;
        omegaMax = alpha * thrustTime;
    }

    /*! If angular rate exceeds limit, increase slew time adding a coasting arc */
    if (omegaMax > SP->slewMaxRate) {
        omegaMax = SP->slewMaxRate;
        totalTime = SP->slewAngle / omegaMax + omegaMax / alpha;
        thrustTime = omegaMax / alpha;
    }

    KinematicProperties* KP = &this->kinematicProperties[index];

    KP->slewRotAxis = SP->slewRotAxis;
    KP->slewAngAcc = alpha;
    KP->slewOmegaMax = omegaMax;
    KP->slewTotalTime = totalTime;
    KP->slewThrustTime = thrustTime;
}

/*! Define this method to compute the rate and acceleration as function of time
    @return ReferenceMotionOutput
    */
ReferenceMotionOutput SunSearchAlgorithm::computeReferenceMotion(const uint64_t currentSimNanos,
																 const uint32_t index) const {
    float zeroTime = 0;
    for (uint32_t i = 0; i < index; ++i) {
        zeroTime += this->kinematicProperties[i].slewTotalTime;
    }
    const float localSimSeconds = (currentSimNanos - this->resetTime) * NANO2SEC - zeroTime;

    const KinematicProperties KP = this->kinematicProperties[index];
    const uint32_t axis = KP.slewRotAxis - 1;

    Eigen::Vector3f omega_RN{Eigen::Vector3f::Zero()};
    Eigen::Vector3f domega_RN{Eigen::Vector3f::Zero()};

    if (localSimSeconds <= KP.slewThrustTime) {
        omega_RN[axis] = KP.slewOmegaMax * localSimSeconds / KP.slewThrustTime;
        domega_RN[axis] = KP.slewAngAcc;
    } else if (localSimSeconds > KP.slewThrustTime && localSimSeconds < KP.slewTotalTime - KP.slewThrustTime) {
        omega_RN[axis] = KP.slewOmegaMax;
    } else if (localSimSeconds >= KP.slewTotalTime - KP.slewThrustTime && localSimSeconds <= KP.slewTotalTime) {
        omega_RN[axis] = KP.slewOmegaMax * (KP.slewTotalTime - localSimSeconds) / KP.slewThrustTime;
        domega_RN[axis] = -KP.slewAngAcc;
    }

    ReferenceMotionOutput referenceMotion{};
    referenceMotion.omega_RN_B = omega_RN;
    referenceMotion.domega_RN_B = domega_RN;

    return referenceMotion;
}

/**
 * @brief Set the properties of a slew maneuver
 * @param slewPropertiesInput the properties of the slew maneuver
 */
void SunSearchAlgorithm::setSlewProperties(const SlewProperties& slewPropertiesInput) {
    this->slewProperties[this->numberOfSlews] = slewPropertiesInput;
    this->numberOfSlews += 1;
}

/**
 * @brief Modify the properties of a slew maneuver
 * @param slewPropertiesInput the properties of the slew maneuver
 * @param index index of the slew maneuver
 */
void SunSearchAlgorithm::modifySlewProperties(const SlewProperties& slewPropertiesInput, const uint32_t index) {
    this->slewProperties[index] = slewPropertiesInput;
}

/**
 * @brief Get the properties of a slew maneuver
 * @param index index of the slew maneuver
 * @return SlewProperties the properties of the slew maneuver
 */
SlewProperties SunSearchAlgorithm::getSlewProperties(const uint32_t index) const { return this->slewProperties[index]; }
