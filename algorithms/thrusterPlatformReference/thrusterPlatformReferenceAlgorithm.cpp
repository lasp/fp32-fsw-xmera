#include "thrusterPlatformReferenceAlgorithm.h"

#include <math.h>
#include <numbers>

#include "utilities/fsw/rigidBodyKinematics.hpp"
#include "utilities/fsw/timeConstants.h"

namespace {
constexpr double kZeroTolerance = 1e-12;  // module tolerance for treating a quantity as zero

/*! Compute the first rotation that makes the thrust direction parallel to the center-of-mass direction. */
Eigen::Matrix3d tprComputeFirstRotation(const Eigen::Vector3d& THat_F, const Eigen::Vector3d& rHat_CM_F) {
    double phi = acos(fmin(fmax(THat_F.dot(rHat_CM_F), -1.0), 1.0));
    Eigen::Vector3d e_phi = THat_F.cross(rHat_CM_F);
    // if phi = pi, e_phi can be any vector perpendicular to THat_F
    if (fabs(phi - std::numbers::pi) < kZeroTolerance) {
        phi = std::numbers::pi;
        if (fabs(THat_F(0)) > kZeroTolerance) {
            e_phi = Eigen::Vector3d(-(THat_F(1) + THat_F(2)) / THat_F(0), 1.0, 1.0);
        } else if (fabs(THat_F(1)) > kZeroTolerance) {
            e_phi = Eigen::Vector3d(1.0, -(THat_F(0) + THat_F(2)) / THat_F(1), 1.0);
        } else {
            e_phi = Eigen::Vector3d(1.0, 1.0, -(THat_F(0) + THat_F(1)) / THat_F(2));
        }
    } else if (fabs(phi) < kZeroTolerance) {
        phi = 0.0;
    }
    e_phi.normalize();
    const Eigen::Vector3d prv = phi * e_phi;
    return prvToDcm(prv);
}

/*! Compute the second rotation that zeroes the offset between the thrust direction and the CM-to-thruster vector. */
Eigen::Matrix3d tprComputeSecondRotation(const Eigen::Vector3d& r_CM_F,
                                         const Eigen::Vector3d& r_TM_F,
                                         const Eigen::Vector3d& r_CT_F,
                                         const Eigen::Vector3d& THat_F) {
    const double a = r_TM_F.norm();
    const double b = r_CM_F.norm();
    const double c1 = r_CT_F.norm();

    double psi = 0.0;
    if (fabs(a) >= kZeroTolerance) {
        const double beta = acos(-fmin(fmax(r_TM_F.dot(THat_F) / a, -1.0), 1.0));
        const double nu = acos(-fmin(fmax(r_TM_F.dot(r_CT_F) / (a * c1), -1.0), 1.0));
        const double c2 = a * cos(beta) + sqrt(b * b - a * a * sin(beta) * sin(beta));
        const double cosGamma1 = (a * a + b * b - c1 * c1) / (2.0 * a * b);
        const double cosGamma2 = (a * a + b * b - c2 * c2) / (2.0 * a * b);
        psi = asin(fmin(fmax((c1 * sin(nu) * cosGamma2 - c2 * sin(beta) * cosGamma1) / b, -1.0), 1.0));
    }

    Eigen::Vector3d e_psi = THat_F.cross(r_CT_F);
    e_psi.normalize();
    const Eigen::Vector3d prv = psi * e_psi;
    return prvToDcm(prv);
}

/*! Compute the third rotation making the frame compliant with the tip-and-tilt platform constraint. */
Eigen::Matrix3d tprComputeThirdRotation(const Eigen::Vector3d& e_theta, const Eigen::Matrix3d& F2M) {
    const double e1 = e_theta(0);
    const double e2 = e_theta(1);
    const double e3 = e_theta(2);

    const double A = 2.0 * (F2M(1, 0) * e2 * e2 + F2M(0, 0) * e1 * e2 + F2M(2, 0) * e2 * e3) - F2M(1, 0);
    const double B = 2.0 * (F2M(2, 0) * e1 - F2M(0, 0) * e3);
    const double C = F2M(1, 0);
    const double Delta = B * B - 4.0 * A * C;

    double theta = 0.0;
    if (fabs(A) < kZeroTolerance) {
        if (fabs(B) < kZeroTolerance) {
            // zero-th order equation has no solution; the minimum problem is solved by theta = pi
            theta = std::numbers::pi;
        } else {
            // first order equation
            theta = 2.0 * atan(-C / B);
        }
    } else if (Delta < 0.0) {
        // second order equation has no solution; find the best solution of the minimum problem
        double t = 0.0;
        if (fabs(B) >= kZeroTolerance) {
            const double q = (A - C) / B;
            const double t1 = q + sqrt(q * q + 1.0);
            const double t2 = q - sqrt(q * q + 1.0);
            const double y1 = (A * t1 * t1 + B * t1 + C) / (1.0 + t1 * t1);
            const double y2 = (A * t2 * t2 + B * t2 + C) / (1.0 + t2 * t2);
            // choose the root that yields the smaller function value
            t = (fabs(y2) < fabs(y1)) ? t2 : t1;
        }
        theta = 2.0 * atan(t);
        const double y = (A * t * t + B * t + C) / (1.0 + t * t);
        // check whether the absolute function minimum is at theta = pi
        if (fabs(A) < fabs(y)) {
            theta = std::numbers::pi;
        }
    } else {
        const double t1 = (-B + sqrt(Delta)) / (2.0 * A);
        const double t2 = (-B - sqrt(Delta)) / (2.0 * A);
        const double t = (fabs(t2) < fabs(t1)) ? t2 : t1;
        theta = 2.0 * atan(t);
    }

    const Eigen::Vector3d prv = theta * e_theta;
    return prvToDcm(prv);
}

/*! Compose the three rotations into the platform-frame DCM that aligns the thruster with the center of mass. */
Eigen::Matrix3d tprComputeFinalRotation(const Eigen::Vector3d& r_CM_M,
                                        const Eigen::Vector3d& r_TM_F,
                                        const Eigen::Vector3d& T_F) {
    const Eigen::Vector3d THat_F = T_F.normalized();
    const Eigen::Vector3d rHat_CM_F = r_CM_M.normalized();  // assume zero initial rotation between F and M

    const Eigen::Matrix3d F1M = tprComputeFirstRotation(THat_F, rHat_CM_F);
    const Eigen::Vector3d r_CM_F = F1M * r_CM_M;
    const Eigen::Vector3d r_CT_F = r_CM_F - r_TM_F;
    const Eigen::Matrix3d F2F1 = tprComputeSecondRotation(r_CM_F, r_TM_F, r_CT_F, THat_F);
    const Eigen::Matrix3d F2M = F2F1 * F1M;
    const Eigen::Vector3d e_theta = (F2M * r_CM_M).normalized();
    const Eigen::Matrix3d F3F2 = tprComputeThirdRotation(e_theta, F2M);
    return F3F2 * F2M;
}
}  // namespace

/*! This method performs a complete reset of the algorithm.  State variables that retain time varying states between
 function calls are reset to their default values.
 @return void
 @param callTime [ns] time the method is called
*/
void ThrusterPlatformReferenceAlgorithm::reset(const uint64_t callTime) {
    this->hsInt_M.setZero();
    this->priorHs_M.setZero();
    this->priorTime = callTime;
}

/*! This method computes the reference platform tip and tilt angles that align the thruster with the system center of
 mass (optionally offset to dump reaction wheel momentum) and the associated body-heading, thruster-torque and
 thruster-configuration quantities.
 @return ThrusterPlatformReferenceOutput reference angles and derived body-frame thruster quantities
 @param in per-cycle inputs read from the input messages
 @param callTime The clock time at which the function was called (nanoseconds)
*/
ThrusterPlatformReferenceOutput ThrusterPlatformReferenceAlgorithm::update(const ThrusterPlatformReferenceInputs& in,
                                                                           const uint64_t callTime) {
    ThrusterPlatformReferenceOutput out{};

    const Eigen::Matrix3d MB = mrpToDcm(this->sigma_MB);         // B to M DCM
    const Eigen::Vector3d r_CB_M = MB * in.r_CB_B;               // position of C w.r.t. B in M-frame coordinates
    const Eigen::Vector3d r_CM_M = r_CB_M + this->r_BM_M;        // position of C w.r.t. M in M-frame coordinates
    const Eigen::Vector3d r_TM_F = this->r_FM_F + in.rThrust_F;  // position of T w.r.t. M in F-frame coordinates
    const Eigen::Vector3d T_F = in.maxThrust * in.tHatThrust_F;  // thrust vector in F-frame coordinates

    Eigen::Matrix3d FM = tprComputeFinalRotation(r_CM_M, r_TM_F, T_F);

    if (this->momentumDumping) {
        // compute net RW momentum in body frame
        Eigen::Vector3d hs_B = Eigen::Vector3d::Zero();
        for (uint32_t i = 0; i < this->rwConfig.numRW; ++i) {
            hs_B += this->rwConfig.JsList(i) * in.wheelSpeeds(i) * this->rwConfig.GsMatrix_B.col(i);
        }
        const Eigen::Vector3d hs_M = MB * hs_B;

        // update the trapezoidal integral of the RW momentum
        const double dt = static_cast<double>(callTime - this->priorTime) * kNano2Sec;
        this->hsInt_M += 0.5 * dt * (this->priorHs_M + hs_M);
        this->priorHs_M = hs_M;
        this->priorTime = callTime;

        // compute the offset vector that shifts the effective CM to produce the desired dumping torque
        const Eigen::Vector3d T_M = FM.transpose() * T_F;
        Eigen::Vector3d H = this->K * hs_M;
        if (this->Ki > 0.0) {
            H += this->Ki * this->hsInt_M;
        }
        const Eigen::Vector3d d_M = -1.0 / T_M.dot(T_M) * T_M.cross(H);

        // recompute the platform rotation about the offset CM
        const Eigen::Vector3d r_CMd_M = r_CM_M + d_M;
        FM = tprComputeFinalRotation(r_CMd_M, r_TM_F, T_F);
    }

    double theta1 = atan2(FM(1, 2), FM(1, 1));
    double theta2 = atan2(FM(2, 0), FM(0, 0));

    // bound the reference angles between the allowed limits
    if ((this->theta1Max > kZeroTolerance) && (theta1 > this->theta1Max)) {
        theta1 = this->theta1Max;
    } else if ((this->theta1Max > kZeroTolerance) && (theta1 < -this->theta1Max)) {
        theta1 = -this->theta1Max;
    }
    if ((this->theta2Max > kZeroTolerance) && (theta2 > this->theta2Max)) {
        theta2 = this->theta2Max;
    } else if ((this->theta2Max > kZeroTolerance) && (theta2 < -this->theta2Max)) {
        theta2 = -this->theta2Max;
    }

    // rebuild the platform DCM with the bounded angles
    FM = eulerAngles123ToDcm(Eigen::Vector3d(theta1, theta2, 0.0));

    out.theta1 = theta1;
    out.theta2 = theta2;

    // mapping between the final platform frame and the body frame
    const Eigen::Matrix3d FB = FM * MB;

    // thruster direction in body frame coordinates
    out.rHat_XB_B = (FB.transpose() * T_F).normalized();

    // thruster torque on the system in body frame coordinates
    const Eigen::Vector3d r_CM_F = FM * r_CM_M;
    const Eigen::Vector3d r_TC_F = r_TM_F - r_CM_F;
    const Eigen::Vector3d Torque_F = T_F.cross(r_TC_F);
    out.torqueRequestBody = FB.transpose() * Torque_F;

    // thruster configuration in body frame coordinates
    const Eigen::Vector3d r_TC_B = FB.transpose() * r_TC_F;
    out.rThrust_B = in.r_CB_B + r_TC_B;
    out.tHatThrust_B = (FB.transpose() * T_F).normalized();
    out.maxThrust = T_F.norm();

    return out;
}
