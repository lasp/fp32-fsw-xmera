#ifndef ORBITAL_MOTION_HPP
#define ORBITAL_MOTION_HPP

#include "freestandingInvalidArgument.h"
#include "safeMath.h"
#include <math.h>
#include <Eigen/Core>
#include <numbers>

namespace orbitalMotion {

inline constexpr int kMaxNumberOfIterations = 200;  //!< Newton-Raphson iteration cap (meanTo*Anomaly solvers)
inline constexpr double kClamp = 7;                 //!< Initial hyperbolic-anomaly guess clamp, radians
inline constexpr double kTolerance = 1e-9;          //!< Convergence/degeneracy tolerance used throughout this file

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

/*! @brief Convert eccentric anomaly to true anomaly for a circular or elliptical orbit.
 *  @param E Eccentric anomaly, radians
 *  @param e Eccentricity; must satisfy 0 <= e < 1, else throws domain_error
 *  @return True anomaly, radians
 */
inline double eccentricToTrueAnomaly(double const E, double const e) {
    if (!(e >= 0.0 && e < 1.0)) FSW_THROW_DOMAIN_ERROR("Eccentricity out of bounds (0 <= e < 1)");
    return 2 * safeAtan2(safeSqrt(1 + e) * safeSin(E / 2), safeSqrt(1 - e) * safeCos(E / 2));
}

/*! @brief Convert eccentric anomaly to mean anomaly via Kepler's equation M = E - e*sin(E).
 *  @param E Eccentric anomaly, radians
 *  @param e Eccentricity; must satisfy 0 <= e < 1, else throws domain_error
 *  @return Mean anomaly, radians
 */
inline double eccentricToMeanAnomaly(double const E, double const e) {
    if (!(e >= 0.0 && e < 1.0)) FSW_THROW_DOMAIN_ERROR("Eccentricity out of bounds (0 <= e < 1)");
    return E - (e * safeSin(E));
}

/*! @brief Convert true anomaly to eccentric anomaly for a circular or elliptical orbit.
 *  @param f True anomaly, radians
 *  @param e Eccentricity; must satisfy 0 <= e < 1, else throws domain_error
 *  @return Eccentric anomaly, radians
 */
inline double trueToEccentricAnomaly(double const f, double const e) {
    if (!(e >= 0.0 && e < 1.0)) FSW_THROW_DOMAIN_ERROR("Eccentricity out of bounds (0 <= e < 1)");
    return 2 * safeAtan2(safeSqrt(1 - e) * safeSin(f / 2), safeSqrt(1 + e) * safeCos(f / 2));
}

/*! @brief Convert true anomaly to mean anomaly (true -> eccentric -> mean).
 *  @param f True anomaly, radians
 *  @param e Eccentricity; must satisfy 0 <= e < 1, else throws domain_error
 *  @return Mean anomaly, radians
 */
inline double trueToMeanAnomaly(double const f, double const e) {
    if (!(e >= 0.0 && e < 1.0)) FSW_THROW_DOMAIN_ERROR("Eccentricity out of bounds (0 <= e < 1)");
    double const eccentric = trueToEccentricAnomaly(f, e);
    return eccentricToMeanAnomaly(eccentric, e);
}

/*! @brief Convert true anomaly to hyperbolic anomaly for a hyperbolic orbit.
 *  @param f True anomaly, radians
 *  @param e Eccentricity; must satisfy e > 1, else throws domain_error
 *  @return Hyperbolic anomaly
 */
inline double trueToHyperbolicAnomaly(double const f, double const e) {
    if (!(e > 1.0)) FSW_THROW_DOMAIN_ERROR("Eccentricity must be > 1 for hyperbolic orbits");
    return 2 * safeAtanH(safeSqrt((e - 1) / (e + 1)) * safeTan(f / 2));
}

/*! @brief Convert hyperbolic anomaly to true anomaly for a hyperbolic orbit.
 *  @param H Hyperbolic anomaly
 *  @param e Eccentricity; must satisfy e > 1, else throws domain_error
 *  @return True anomaly, radians
 */
inline double hyperbolicToTrueAnomaly(double const H, double const e) {
    if (!(e > 1.0)) FSW_THROW_DOMAIN_ERROR("Eccentricity must be > 1 for hyperbolic orbits");
    return 2 * safeAtan(safeSqrt((e + 1) / (e - 1)) * safeTanH(H / 2));
}

/*! @brief Convert hyperbolic anomaly to mean anomaly via N = e*sinh(H) - H.
 *  @param H Hyperbolic anomaly
 *  @param e Eccentricity; must satisfy e > 1, else throws domain_error
 *  @return Mean anomaly (hyperbolic)
 */
inline double hyperbolicToMeanAnomaly(double const H, double const e) {
    if (!(e > 1.0)) FSW_THROW_DOMAIN_ERROR("Eccentricity must be > 1 for hyperbolic orbits");
    return (e * safeSinH(H)) - H;
}

/*! @brief Solve Kepler's equation M = E - e*sin(E) for E via Newton-Raphson, starting from
 *         E = M and clamping each step to [-0.5, 0.5]. Iterates up to
 *         kMaxNumberOfIterations; if convergence to kTolerance is not reached by then, the
 *         current best estimate is returned with no indication that it did not converge.
 *  @param M Mean anomaly, radians
 *  @param e Eccentricity; must satisfy 0 <= e < 1, else throws domain_error
 *  @return Eccentric anomaly, radians (best available estimate)
 */
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

/*! @brief Convert mean anomaly to true anomaly (mean -> eccentric -> true).
 *  @param M Mean anomaly, radians
 *  @param e Eccentricity; must satisfy 0 <= e < 1, else throws domain_error
 *  @return True anomaly, radians
 */
inline double meanToTrueAnomaly(double const M, double const e) {
    if (!(e >= 0.0 && e < 1.0)) FSW_THROW_DOMAIN_ERROR("Eccentricity out of bounds (0 <= e < 1)");
    double const eccentric = meanToEccentricAnomaly(M, e);
    return eccentricToTrueAnomaly(eccentric, e);
}

/*! @brief Solve the hyperbolic Kepler equation N = e*sinh(H) - H for H via Newton-Raphson,
 *         starting from N clamped to [-kClamp, kClamp]. Iterates up to
 *         kMaxNumberOfIterations; if convergence to kTolerance is not reached by then, the
 *         current best estimate is returned with no indication that it did not converge.
 *  @param N Mean anomaly (hyperbolic)
 *  @param e Eccentricity; must satisfy e > 1, else throws domain_error
 *  @return Hyperbolic anomaly (best available estimate)
 */
inline double meanToHyperbolicAnomaly(const double N, const double e) {
    if (!(e > 1.0)) FSW_THROW_DOMAIN_ERROR("Eccentricity must be > 1");
    const int signN = (N > 0 ? 1 : -1);
    double H = fabs(N) > kClamp ? kClamp * static_cast<double>(signN) : N;
    for (int i = 0; i < kMaxNumberOfIterations; ++i) {
        const double dH = (e * safeSinH(H) - H - N) / (e * safeCosH(H) - 1);
        H -= fmax(-0.5, fmin(0.5, dH));  // Clamp step size
        if (fabs(dH) < kTolerance) {
            break;
        }
    }
    return H;
}

/*! @brief Convert classical orbital elements to a Cartesian position/velocity state.
 *         p is computed differently for parabolic orbit, indicated by semiMajorAxis = 0.
 *  @param mu Gravitational parameter of the central body
 *  @param elements Classical orbital elements
 *  @return Cartesian position and velocity
 */
inline CartesianState elementsToCartesianState(double const mu, const ClassicalElements& elements) {
    double const a = elements.semiMajorAxis;
    double const e = elements.eccentricity;
    double const i = elements.inclination;
    double const Omega = elements.rightAscensionAscendingNode;
    double const omega = elements.argPeriapsis;
    double const f = elements.trueAnomaly;

    double const p = a != 0.0 ? a * (1 - e * e) : elements.radiusPeriapsis * (1 + e);
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

    CartesianState state{};
    state.position = rVec;
    state.velocity = Eigen::Vector3d(vx, vy, vz);
    return state;
}

/*! @brief Convert a Cartesian position/velocity state to classical orbital elements.
 *         Rectilinear orbits (h < kTolerance) get inclination = 0; parabolic orbits
 *         (|e - 1| < kTolerance) get radiusApoapsis = 0. RAAN, argument of periapsis, and
 *         inclination are ill-defined for equatorial/circular/polar orbits but degrade to
 *         a defined value (0 via safeAtan2(0, 0), or a rail via safeAcos) rather than NaN.
 *  @param mu Gravitational parameter of the central body
 *  @param rVec Position vector
 *  @param vVec Velocity vector
 *  @return Classical orbital elements
 */
inline ClassicalElements cartesianStateToElements(const double mu,
                                                  const Eigen::Vector3d& rVec,
                                                  const Eigen::Vector3d& vVec) {
    const double r = rVec.stableNorm();
    const double v = vVec.stableNorm();
    const Eigen::Vector3d hVec = rVec.cross(vVec);
    const double h = hVec.stableNorm();
    const Eigen::Vector3d nVec = Eigen::Vector3d::UnitZ().cross(hVec);
    const Eigen::Vector3d eVec = (((v * v) - (mu / r)) * rVec - (rVec.dot(vVec)) * vVec) / mu;

    ClassicalElements elements{};

    elements.radiusMagnitude = r;
    elements.eccentricity = eVec.stableNorm();
    if (h < kTolerance) {
        elements.inclination = 0.0;  // rectilinear orbit
    } else {
        elements.inclination = safeAcos(hVec(2) / h);
    }
    elements.alpha = (2 / r) - (v * v / mu);
    elements.semiMajorAxis = fabs(elements.alpha) > kTolerance ? 1 / elements.alpha : 0.0;

    const double Omega = safeAtan2(nVec(1), nVec(0));
    elements.rightAscensionAscendingNode = Omega < 0 ? Omega + (2 * std::numbers::pi) : Omega;

    const double omega = safeAtan2(nVec.cross(eVec).dot(hVec.stableNormalized()), nVec.dot(eVec));
    elements.argPeriapsis = omega < 0 ? omega + (2 * std::numbers::pi) : omega;

    const double f = safeAtan2(eVec.cross(rVec).dot(hVec.stableNormalized()), eVec.dot(rVec));
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
