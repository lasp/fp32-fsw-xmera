#include "solarArrayReference.h"
#include <math.h>
#include <numbers>
#include <stdexcept>
#include <string.h>

#include "utilities/timeConstants.h"

#include <architecture/utilities/eigenSupport.h>
#include <architecture/utilities/rigidBodyKinematics.hpp>

const double epsilon = 1e-12;  // module tolerance for zero

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param callTime [ns] time the method is called
*/
void SolarArrayReference::reset(uint64_t callTime) {
    if (!this->attNavInMsg.isLinked()) {
        throw std::invalid_argument("solarArrayReference.attNavInMsg wasn't connected.");
    }
    if (!this->attRefInMsg.isLinked()) {
        throw std::invalid_argument("solarArrayReference.attRefInMsg wasn't connected.");
    }
    if (!this->hingedRigidBodyInMsg.isLinked()) {
        throw std::invalid_argument("solarArrayReference.hingedRigidBodyInMsg wasn't connected.");
    }
    this->count = 0;
}

/*! This method computes the updated rotation angle reference based on current attitude, reference attitude, and current
 rotation angle
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
*/
void SolarArrayReference::updateState(uint64_t callTime) {
    /*! - Create and assign buffer messages */
    NavAttMsgPayload attNavIn = this->attNavInMsg();
    AttRefMsgPayload attRefIn = this->attRefInMsg();
    HingedRigidBodyMsgPayload hingedRigidBodyIn = this->hingedRigidBodyInMsg();
    HingedRigidBodyMsgPayload hingedRigidBodyRefOut = {};

    /*! read Sun direction in B frame from the attNav message and map it to R frame */
    const Eigen::Vector3d rHat_SB_B = cArrayToEigenVector(attNavIn.vehSunPntBdy).normalized();
    Eigen::Vector3d rHat_SB_R;
    switch (this->attitudeFrame) {
        case referenceFrame: {
            const Eigen::Matrix3d dcm_BN = mrpToDcm(cArrayToEigenVector(attNavIn.sigma_BN));
            const Eigen::Matrix3d dcm_RN = mrpToDcm(cArrayToEigenVector(attRefIn.sigma_RN));
            const Eigen::Matrix3d dcm_RB = dcm_RN * dcm_BN.transpose();
            rHat_SB_R = dcm_RB * rHat_SB_B;
            break;
        }

        case bodyFrame:
            rHat_SB_R = rHat_SB_B;
            break;

        default:
            throw std::invalid_argument("solarArrayReference.attitudeFrame input can be either 0 or 1.");
    }

    /*! compute solar array frame axes at zero rotation */
    const Eigen::Vector3d a1 = this->a1Hat_B.normalized();
    const Eigen::Vector3d a3 = (a1.cross(this->a2Hat_B)).normalized();
    const Eigen::Vector3d a2 = (a3.cross(a1)).normalized();

    /*! compute solar array reference frame axes at zero rotation */
    const double dotP = a1.dot(rHat_SB_R);
    Eigen::Vector3d a2Hat_R = rHat_SB_R - dotP * a1;
    const double a2Hat_R_norm = a2Hat_R.norm();

    /*! compute current rotation angle thetaC from input msg */
    const double sinThetaC = sin(hingedRigidBodyIn.theta);
    const double cosThetaC = cos(hingedRigidBodyIn.theta);
    const double thetaC = atan2(sinThetaC, cosThetaC);  // clip theta current between 0 and 2*pi

    /*! compute reference angle and store in buffer msg */
    if (a2Hat_R_norm < epsilon) {
        // if norm(a2Hat_R) = 0, reference coincides with current angle
        hingedRigidBodyRefOut.theta = hingedRigidBodyIn.theta;
    } else {
        a2Hat_R.normalize();
        const Eigen::Vector3d a1Hat_R = a2.cross(a2Hat_R);
        double thetaR = acos(fmin(fmax(a2.dot(a2Hat_R), -1.0), 1.0));
        // if a1 and a1Hat_R are opposite, take the negative of thetaR
        if (a1.dot(a1Hat_R) < 0) {
            thetaR = -thetaR;
        }
        // always make the absolute difference |thetaR-thetaC| smaller that 2*pi
        if (thetaR - thetaC > std::numbers::pi) {
            hingedRigidBodyRefOut.theta = hingedRigidBodyIn.theta + thetaR - thetaC - 2 * std::numbers::pi;
        } else if (thetaR - thetaC < -std::numbers::pi) {
            hingedRigidBodyRefOut.theta = hingedRigidBodyIn.theta + thetaR - thetaC + 2 * std::numbers::pi;
        } else {
            hingedRigidBodyRefOut.theta = hingedRigidBodyIn.theta + thetaR - thetaC;
        }
    }

    /*! implement finite differences to compute thetaDotR */
    double dt;
    if (this->count == 0) {
        hingedRigidBodyRefOut.thetaDot = 0;
    } else {
        dt = static_cast<double>(callTime - this->priorT) * kNano2Sec;
        hingedRigidBodyRefOut.thetaDot = (hingedRigidBodyRefOut.theta - this->priorThetaR) / dt;
    }
    // update stored variables
    this->priorThetaR = hingedRigidBodyRefOut.theta;
    this->priorT = callTime;
    this->count += 1;

    /* write output message */
    this->hingedRigidBodyRefOutMsg.write(&hingedRigidBodyRefOut, this->moduleID, callTime);
}
