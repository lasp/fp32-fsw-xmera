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

#include <Eigen/Core>

struct CartesianState {
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
};

class ClassicalElements {
   public:
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

class OrbitalMotion {
   public:
    static double eccentricToTrueAnomaly(double E, double e);
    static double eccentricToMeanAnomaly(double E, double e);
    static double trueToEccentricAnomaly(double f, double e);
    static double trueToHyperbolicAnomaly(double f, double e);
    static double trueToMeanAnomaly(double f, double e);
    static double hyperbolicToTrueAnomaly(double H, double e);
    static double hyperbolicToMeanAnomaly(double H, double e);
    static double meanToEccentricAnomaly(double M, double e);
    static double meanToTrueAnomaly(double M, double e);
    static double meanToHyperbolicAnomaly(double N, double e);
    static CartesianState elementsToCartesianState(double mu, const ClassicalElements& elements);
    static ClassicalElements cartesianStateToElements(double mu,
                                                      const Eigen::Vector3d& rVec,
                                                      const Eigen::Vector3d& vVec);
};

#endif
