#include "triadAlgorithm.h"

#include <cmath>
#include <numbers>
#include <stdexcept>

#include <Eigen/Core>

#include <architecture/utilities/rigidBodyKinematics.hpp>

static constexpr double kSpeParallelThresholdDeg = 0.5;
static constexpr double kRadToDeg = 180.0 / std::numbers::pi;

static double SPE_angle(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2) {
    const double dot = v1.dot(v2);
    const double cross = v1.x() * v2.y() - v1.y() * v2.x();

    double angle = std::acos(std::clamp(dot / (v1.norm() * v2.norm()), -1.0, 1.0));
    angle = angle * kRadToDeg;

    if (cross < 0) {
        angle = -angle;
    }

    return angle;
}

Eigen::Vector3d TriadAlgorithm::update(const Eigen::Vector3d& rHat_SB_N,
                                       const Eigen::Vector3d& hReqHat_N,
                                       const Eigen::Vector3d& hRefHat_B) const {
    if (const double SPE = SPE_angle(rHat_SB_N, hReqHat_N); std::abs(SPE) < kSpeParallelThresholdDeg) {
        throw std::runtime_error("sun and earth reference vectors are parallel, Triad can not be used");
    }

    const Eigen::Vector3d a1 = this->a1Hat_B.normalized();

    Eigen::Matrix3d RD;
    const Eigen::Vector3d r2 = hRefHat_B;
    const Eigen::Vector3d r3 = a1.cross(hRefHat_B).normalized();
    const Eigen::Vector3d r1 = r2.cross(r3);
    RD.col(0) = r1;
    RD.col(1) = r2;
    RD.col(2) = r3;

    Eigen::Matrix3d BD;
    const Eigen::Vector3d n2 = hReqHat_N;
    const Eigen::Vector3d n1 = rHat_SB_N.cross(hReqHat_N).normalized();
    const Eigen::Vector3d n3 = n1.cross(n2);
    BD.col(0) = n1;
    BD.col(1) = n2;
    BD.col(2) = n3;

    const Eigen::Matrix3d RN = RD * BD.transpose();

    return dcmToMrp(RN);
}

void TriadAlgorithm::setA1Hat_B(const Eigen::Vector3d& a1Hat_B) { this->a1Hat_B = a1Hat_B; }
Eigen::Vector3d TriadAlgorithm::getA1Hat_B() const { return this->a1Hat_B; }
