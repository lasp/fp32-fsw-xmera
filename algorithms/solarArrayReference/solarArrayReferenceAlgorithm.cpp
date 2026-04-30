#include "solarArrayReferenceAlgorithm.h"
#include "utilities/freestandingInvalidArgument.h"
#include <math.h>
#include <numbers>

#include "architecture/utilities/rigidBodyKinematics.hpp"
#include "utilities/timeConstants.h"

const double epsilon = 1e-12;  // module tolerance for zero

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
*/
void SolarArrayReferenceAlgorithm::reset() { this->count = 0; }

/*! This method computes the updated rotation angle reference based on current attitude, reference attitude, and current
 rotation angle
 @return SolarArrayReferenceOutput containing reference theta and thetaDot
 @param sigma_BN body attitude MRP with respect to inertial frame
 @param sigma_RN reference attitude MRP with respect to inertial frame
 @param vehSunPntBdy Sun pointing vector in body frame
 @param theta current panel angular displacement [rad]
 @param callTime The clock time at which the function was called (nanoseconds)
*/
SolarArrayReferenceOutput SolarArrayReferenceAlgorithm::update(const Eigen::Vector3d& sigma_BN,
                                                               const Eigen::Vector3d& sigma_RN,
                                                               const Eigen::Vector3d& vehSunPntBdy,
                                                               double theta,
                                                               uint64_t callTime) {
    SolarArrayReferenceOutput output{};

    /*! read Sun direction in B frame and map it to R frame */
    const Eigen::Vector3d rHat_SB_B = vehSunPntBdy.normalized();
    Eigen::Vector3d rHat_SB_R;
    switch (this->attitudeFrame) {
        case referenceFrame: {
            const Eigen::Matrix3d dcm_BN = mrpToDcm(sigma_BN);
            const Eigen::Matrix3d dcm_RN = mrpToDcm(sigma_RN);
            const Eigen::Matrix3d dcm_RB = dcm_RN * dcm_BN.transpose();
            rHat_SB_R = dcm_RB * rHat_SB_B;
            break;
        }

        case bodyFrame:
            rHat_SB_R = rHat_SB_B;
            break;

        default:
            FSW_THROW_INVALID_ARGUMENT("solarArrayReference.attitudeFrame input can be either 0 or 1.");
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
    const double sinThetaC = sin(theta);
    const double cosThetaC = cos(theta);
    const double thetaC = atan2(sinThetaC, cosThetaC);  // clip theta current between 0 and 2*pi

    /*! compute reference angle and store in output */
    if (a2Hat_R_norm < epsilon) {
        // if norm(a2Hat_R) = 0, reference coincides with current angle
        output.theta = theta;
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
            output.theta = theta + thetaR - thetaC - 2 * std::numbers::pi;
        } else if (thetaR - thetaC < -std::numbers::pi) {
            output.theta = theta + thetaR - thetaC + 2 * std::numbers::pi;
        } else {
            output.theta = theta + thetaR - thetaC;
        }
    }

    /*! implement finite differences to compute thetaDotR */
    if (this->count == 0) {
        output.thetaDot = 0;
    } else {
        const double dt = static_cast<double>(callTime - this->priorT) * kNano2Sec;
        output.thetaDot = (output.theta - this->priorThetaR) / dt;
    }
    // update stored variables
    this->priorThetaR = output.theta;
    this->priorT = callTime;
    this->count += 1;

    return output;
}
