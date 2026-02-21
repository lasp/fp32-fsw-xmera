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

#include "orbitalMotion.hpp"
#include "safeMath.h"
#include <Eigen/Geometry>
#include <numbers>

constexpr double toleranceF32 = 1e-6;
constexpr double tolerance = 1e-9;
constexpr int maxNumberOfIterations = 200;
constexpr float clamp = 7;

/**
 * @brief Converts eccentric anomaly to true anomaly.
 * @param E Eccentric anomaly in radians.
 * @param e Orbital eccentricity.
 * @return True anomaly in radians.
 */
float OrbitalMotion::eccentricToTrueAnomalyF32(float const E, float const e) {
    assert((e >= 0.0 || e < 1.0) && "Eccentricity out of bounds (0 <= e < 1)");
    return 2 * safeAtan2f(safeSqrtf(1 + e) * safeSinf(E / 2), safeSqrtf(1 - e) * safeCosf(E / 2));
}

/**
 * @brief Converts eccentric anomaly to mean anomaly.
 * @param E Eccentric anomaly in radians.
 * @param e Orbital eccentricity.
 * @return Mean anomaly in radians.
 */
float OrbitalMotion::eccentricToMeanAnomalyF32(float const E, float const e) {
    assert((e >= 0.0 || e < 1.0) && "Eccentricity out of bounds (0 <= e < 1)");
    return E - (e * safeSinf(E));
}

/**
 * @brief Converts true anomaly to eccentric anomaly.
 * @param f True anomaly in radians.
 * @param e Orbital eccentricity.
 * @return Eccentric anomaly in radians.
 */
float OrbitalMotion::trueToEccentricAnomalyF32(float const f, float const e) {
    assert((e >= 0.0 || e < 1.0) && "Eccentricity out of bounds (0 <= e < 1)");
    return 2 * safeAtan2f(safeSqrtf(1 - e) * safeSinf(f / 2), safeSqrtf(1 + e) * safeCosf(f / 2));
}

/**
 * @brief Convert true anomaly to mean anomaly
 * @param f True anomaly in radians
 * @param e Orbital eccentricity (0 <= e < 1).
 * @return Mean anomaly in radians.
 */
float OrbitalMotion::trueToMeanAnomalyF32(float const f, float const e) {
    assert((e >= 0.0 || e < 1.0) && "Eccentricity out of bounds (0 <= e < 1)");
    float const eccentric = trueToEccentricAnomalyF32(f, e);
    return eccentricToMeanAnomalyF32(eccentric, e);
}

/**
 * @brief Converts true anomaly to hyperbolic anomaly.
 * @param f True anomaly in radians.
 * @param e Orbital eccentricity (> 1).
 * @return Hyperbolic anomaly in radians.
 */
float OrbitalMotion::trueToHyperbolicAnomalyF32(float const f, float const e) {
    assert(e > 1.0 && "Eccentricity must be > 1 for hyperbolic orbits");
    return 2 * safeAtanHf(safeSqrtf((e - 1) / (e + 1)) * safeTanf(f / 2));
}

/**
 * @brief Converts hyperbolic anomaly to true anomaly.
 * @param H Hyperbolic anomaly in radians.
 * @param e Orbital eccentricity (> 1).
 * @return True anomaly in radians.
 */
float OrbitalMotion::hyperbolicToTrueAnomalyF32(float const H, float const e) {
    assert(e > 1.0 && "Eccentricity must be > 1 for hyperbolic orbits");
    return 2 * safeAtanf(safeSqrtf((e + 1) / (e - 1)) * safeTanHf(H / 2));
}

/**
 * @brief Converts hyperbolic anomaly to mean hyperbolic anomaly.
 * @param H Hyperbolic anomaly in radians.
 * @param e Orbital eccentricity (> 1).
 * @return Mean hyperbolic anomaly in radians.
 */
float OrbitalMotion::hyperbolicToMeanAnomalyF32(float const H, float const e) {
    assert(e > 1.0 && "Eccentricity must be > 1 for hyperbolic orbits");
    return (e * safeSinHf(H)) - H;
}

/**
 * @brief Convert mean anomaly to eccentric anomaly
 * @param M Mean anomaly in radians.
 * @param e Orbital eccentricity (0 <= e < 1).
 * @return Eccentric anomaly in radians.
 */
float OrbitalMotion::meanToEccentricAnomalyF32(float M, float e) {
    assert((e >= 0.0 || e < 1.0) && "Eccentricity out of bounds (0 <= e < 1)");
    float E = M;
    for (int i = 0; i < maxNumberOfIterations; ++i) {
        float const dE = (E - e * safeSinf(E) - M) / (1 - e * safeCosf(E));
        E -= dE;
        if (std::abs(dE) < toleranceF32) {
            break;
        }
    }
    return E;
}

/**
 * @brief Convert mean anomaly to true anomaly
 * @param M Mean anomaly in radians.
 * @param e Orbital eccentricity (0 <= e < 1).
 * @return True anomaly in radians.
 */
float OrbitalMotion::meanToTrueAnomalyF32(float const M, float const e) {
    assert((e >= 0.0 || e < 1.0) && "Eccentricity out of bounds (0 <= e < 1)");
    float const eccentric = meanToEccentricAnomalyF32(M, e);
    return eccentricToTrueAnomalyF32(eccentric, e);
}

/**
 * @brief Mean hyperbolic anomaly to hyperbolic anomaly
 * @param N Mean hyperbolic anomaly in radians.
 * @param e Orbital eccentricity (> 1).
 * @return Hyperbolic anomaly in radians.
 */
float OrbitalMotion::meanToHyperbolicAnomalyF32(const float N, const float e) {
    assert(e > 1.0 && "Eccentricity must be > 1");
    const int signN = (N > 0 ? 1 : -1);
    float H = std::abs(N) > clamp ? clamp * static_cast<float>(signN) : N;
    for (int i = 0; i < maxNumberOfIterations; ++i) {
        const float dH = (e * safeSinHf(H) - H - N) / (e * safeCosHf(H) - 1);
        H -= dH;
        if (std::abs(dH) < toleranceF32) {
            break;
        }
    }
    return H;
}

/**
 * @brief Converts orbital elements to position and velocity vectors.
 * @param mu Gravitational parameter (km^3/s^2).
 * @param elements Classical orbital elements (a, e, i, Omega, omega, f).
 * @param CartesianState Output: position and velocity vectors in km and km/s.
 */
CartesianState OrbitalMotion::elementsToCartesianStateF32(double const mu, const ClassicalElementsF32& elements) {
    double const a = elements.semiMajorAxis;
    float const e = elements.eccentricity;
    float const i = elements.inclination;
    float const Omega = elements.rightAscensionAscendingNode;
    float const omega = elements.argPeriapsis;
    float const f = elements.trueAnomaly;

    double const p = a * (1 - e * e);
    double const r = p / (1 + e * safeCosf(f));
    double const h = safeSqrt(mu * p);

    float const cos_O = safeCosf(Omega);
    float const sin_O = safeSinf(Omega);
    float const cos_o = safeCosf(omega);
    float const sin_o = safeSinf(omega);
    float const cos_i = safeCosf(i);
    float const sin_i = safeSinf(i);
    float const cos_f = safeCosf(f);
    float const sin_f = safeSinf(f);

    float const cos_theta = (cos_o * cos_f) - (sin_o * sin_f);
    float const sin_theta = (sin_o * cos_f) + (cos_o * sin_f);

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

/**
 * @brief Converts position and velocity vectors to classical orbital elements.
 * @param mu Gravitational parameter (km^3/s^2).
 * @param rVec Position vector in km.
 * @param vVec Velocity vector in km/s.
 * @return elements : classical orbital elements (a, e, i, Omega, omega, f).
 */
ClassicalElementsF32 OrbitalMotion::cartesianStateToElementsF32(const double mu,
                                                                const Eigen::Vector3d& rVec,
                                                                const Eigen::Vector3d& vVec) {
    const double r = rVec.norm();
    const double v = vVec.norm();
    const Eigen::Vector3d hVec = rVec.cross(vVec);
    const double h = hVec.norm();
    const Eigen::Vector3d nVec = Eigen::Vector3d::UnitZ().cross(hVec);
    const Eigen::Vector3d eVec = (((v * v) - (mu / r)) * rVec - (rVec.dot(vVec)) * vVec) / mu;

    ClassicalElementsF32 elements{};

    elements.radiusMagnitude = r;
    elements.eccentricity = static_cast<float>(eVec.norm());
    if (h > tolerance) {
        elements.inclination = static_cast<float>(safeAcos(hVec(2) / h));
    } else {
        elements.inclination = 0.0F;
    }
    elements.alpha = (2 / r) - (v * v / mu);
    elements.semiMajorAxis = std::abs(elements.alpha) > tolerance ? 1 / elements.alpha : 0.0;

    auto Omega = static_cast<float>(safeAtan2(nVec(1), nVec(0)));
    elements.rightAscensionAscendingNode = Omega < 0 ? static_cast<float>(Omega + (2 * std::numbers::pi)) : Omega;

    auto omega = static_cast<float>(safeAtan2(nVec.cross(eVec).dot(hVec.normalized()), nVec.dot(eVec)));
    elements.argPeriapsis = omega < 0 ? static_cast<float>(omega + (2 * std::numbers::pi)) : omega;

    auto f = static_cast<float>(safeAtan2(eVec.cross(rVec).dot(hVec.normalized()), eVec.dot(rVec)));
    elements.trueAnomaly = f < 0 ? static_cast<float>(f + (2 * std::numbers::pi)) : f;

    elements.radiusPeriapsis = h * h / mu / (1 + elements.eccentricity);
    if (std::abs(elements.eccentricity - 1) < tolerance) {
        elements.radiusApoapsis = 0.0F;
    } else {
        elements.radiusApoapsis = h * h / mu / (1 - elements.eccentricity);
    }
    return elements;
}
