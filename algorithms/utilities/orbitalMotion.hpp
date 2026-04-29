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

#ifndef ORBITAL_MOTION_HPP
#define ORBITAL_MOTION_HPP

#include "freestandingInvalidArgument.h"
#include "safeMath.h"
#include <math.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <numbers>

namespace orbitalMotion {

inline constexpr int kMaxNumberOfIterations = 200;
inline constexpr double kClamp = 7;
inline constexpr double kTolerance = 1e-9;

struct CartesianState {
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
};

struct ClassicalElements {
    double semiMajorAxis = 0;
    double eccentricity = 0;
    double inclination = 0;
    double rightAscensionAscendingNode = 0;
    double argPeriapsis = 0;
    double trueAnomaly = 0;
    double radiusMagnitude = 0;
    double alpha = 0;
    double radiusPeriapsis = 0;
    double radiusApoapsis = 0;
};

inline double eccentricToTrueAnomaly(double const E, double const e) {
    if (!(e >= 0.0 && e < 1.0)) FSW_THROW_DOMAIN_ERROR("Eccentricity out of bounds (0 <= e < 1)");
    return 2 * safeAtan2(safeSqrt(1 + e) * safeSin(E / 2), safeSqrt(1 - e) * safeCos(E / 2));
}

inline double eccentricToMeanAnomaly(double const E, double const e) {
    if (!(e >= 0.0 && e < 1.0)) FSW_THROW_DOMAIN_ERROR("Eccentricity out of bounds (0 <= e < 1)");
    return E - (e * safeSin(E));
}

inline double trueToEccentricAnomaly(double const f, double const e) {
    if (!(e >= 0.0 && e < 1.0)) FSW_THROW_DOMAIN_ERROR("Eccentricity out of bounds (0 <= e < 1)");
    return 2 * safeAtan2(safeSqrt(1 - e) * safeSin(f / 2), safeSqrt(1 + e) * safeCos(f / 2));
}

inline double trueToMeanAnomaly(double const f, double const e) {
    if (!(e >= 0.0 && e < 1.0)) FSW_THROW_DOMAIN_ERROR("Eccentricity out of bounds (0 <= e < 1)");
    double const eccentric = trueToEccentricAnomaly(f, e);
    return eccentricToMeanAnomaly(eccentric, e);
}

inline double trueToHyperbolicAnomaly(double const f, double const e) {
    if (!(e > 1.0)) FSW_THROW_DOMAIN_ERROR("Eccentricity must be > 1 for hyperbolic orbits");
    return 2 * safeAtanH(safeSqrt((e - 1) / (e + 1)) * safeTan(f / 2));
}

inline double hyperbolicToTrueAnomaly(double const H, double const e) {
    if (!(e > 1.0)) FSW_THROW_DOMAIN_ERROR("Eccentricity must be > 1 for hyperbolic orbits");
    return 2 * safeAtan(safeSqrt((e + 1) / (e - 1)) * safeTanH(H / 2));
}

inline double hyperbolicToMeanAnomaly(double const H, double const e) {
    if (!(e > 1.0)) FSW_THROW_DOMAIN_ERROR("Eccentricity must be > 1 for hyperbolic orbits");
    return (e * safeSinH(H)) - H;
}

inline double meanToEccentricAnomaly(double M, double e) {
    if (!(e >= 0.0 && e < 1.0)) FSW_THROW_DOMAIN_ERROR("Eccentricity out of bounds (0 <= e < 1)");
    double E = M;
    for (int i = 0; i < kMaxNumberOfIterations; ++i) {
        double const dE = (E - e * safeSin(E) - M) / (1 - e * safeCos(E));
        E -= fmax(-0.5, fmin(0.5, dE));  // Clamp step size in case of near parabolic orbits
        if (fabs(dE) < kTolerance) {
            break;
        }
    }
    return E;
}

inline double meanToTrueAnomaly(double const M, double const e) {
    if (!(e >= 0.0 && e < 1.0)) FSW_THROW_DOMAIN_ERROR("Eccentricity out of bounds (0 <= e < 1)");
    double const eccentric = meanToEccentricAnomaly(M, e);
    return eccentricToTrueAnomaly(eccentric, e);
}

inline double meanToHyperbolicAnomaly(const double N, const double e) {
    if (!(e > 1.0)) FSW_THROW_DOMAIN_ERROR("Eccentricity must be > 1");
    const int signN = (N > 0 ? 1 : -1);
    double H = fabs(N) > kClamp ? kClamp * static_cast<double>(signN) : N;
    for (int i = 0; i < kMaxNumberOfIterations; ++i) {
        const double dH = (e * safeSinH(H) - H - N) / (e * safeCosH(H) - 1);
        H -= dH;
        if (fabs(dH) < kTolerance) {
            break;
        }
    }
    return H;
}

inline CartesianState elementsToCartesianState(double const mu, const ClassicalElements& elements) {
    double const a = elements.semiMajorAxis;
    double const e = elements.eccentricity;
    double const i = elements.inclination;
    double const Omega = elements.rightAscensionAscendingNode;
    double const omega = elements.argPeriapsis;
    double const f = elements.trueAnomaly;

    double const p = a * (1 - e * e);
    double const r = p / (1 + e * safeCos(f));
    double const h = safeSqrt(mu * p);

    double const cos_O = safeCos(Omega);
    double const sin_O = safeSin(Omega);
    double const cos_o = safeCos(omega);
    double const sin_o = safeSin(omega);
    double const cos_i = safeCos(i);
    double const sin_i = safeSin(i);
    double const cos_f = safeCos(f);
    double const sin_f = safeSin(f);

    double const cos_theta = (cos_o * cos_f) - (sin_o * sin_f);
    double const sin_theta = (sin_o * cos_f) + (cos_o * sin_f);

    Eigen::Vector3d rVec{};
    rVec(0) = r * (cos_O * cos_theta - sin_O * sin_theta * cos_i);
    rVec(1) = r * (sin_O * cos_theta + cos_O * sin_theta * cos_i);
    rVec(2) = r * (sin_theta * sin_i);

    double const vx = -mu / h * (cos_O * (sin_theta + e * sin_o) + sin_O * (cos_theta + e * cos_o) * cos_i);
    double const vy = -mu / h * (sin_O * (sin_theta + e * sin_o) - cos_O * (cos_theta + e * cos_o) * cos_i);
    double const vz = mu / h * (cos_theta + e * cos_o) * sin_i;

    const Eigen::Vector3d vVec = Eigen::Vector3d(vx, vy, vz);

    CartesianState state{};
    state.position = rVec;
    state.velocity = vVec;

    return state;
}

inline ClassicalElements cartesianStateToElements(const double mu,
                                                  const Eigen::Vector3d& rVec,
                                                  const Eigen::Vector3d& vVec) {
    const double r = rVec.norm();
    const double v = vVec.norm();
    const Eigen::Vector3d hVec = rVec.cross(vVec);
    const double h = hVec.norm();
    const Eigen::Vector3d nVec = Eigen::Vector3d::UnitZ().cross(hVec);
    const Eigen::Vector3d eVec = (((v * v) - (mu / r)) * rVec - (rVec.dot(vVec)) * vVec) / mu;

    ClassicalElements elements{};

    elements.radiusMagnitude = r;
    elements.eccentricity = eVec.norm();
    if (h < kTolerance) {
        elements.inclination = 0.0;  // rectilinear orbit
    } else {
        elements.inclination = std::acos(hVec(2) / h);
    }
    elements.alpha = (2 / r) - (v * v / mu);
    elements.semiMajorAxis = fabs(elements.alpha) > kTolerance ? 1 / elements.alpha : 0.0;

    const double Omega = safeAtan2(nVec(1), nVec(0));
    elements.rightAscensionAscendingNode = Omega < 0 ? Omega + (2 * std::numbers::pi) : Omega;

    const double omega = safeAtan2(nVec.cross(eVec).dot(hVec.normalized()), nVec.dot(eVec));
    elements.argPeriapsis = omega < 0 ? omega + (2 * std::numbers::pi) : omega;

    const double f = safeAtan2(eVec.cross(rVec).dot(hVec.normalized()), eVec.dot(rVec));
    elements.trueAnomaly = f < 0 ? f + (2 * std::numbers::pi) : f;

    elements.radiusPeriapsis = h * h / mu / (1 + elements.eccentricity);
    if (fabs(elements.eccentricity - 1) < kTolerance) {
        elements.radiusApoapsis = 0.0;  // parabolic orbit
    } else {
        elements.radiusApoapsis = h * h / mu / (1 - elements.eccentricity);
    }
    return elements;
}

}  // namespace orbitalMotion

#endif
