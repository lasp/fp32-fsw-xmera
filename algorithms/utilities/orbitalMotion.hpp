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

#ifndef ORBITAL_MOTION_FP32_HPP
#define ORBITAL_MOTION_FP32_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

struct CartesianState {
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
};

class ClassicalElementsF32 {
   public:
    double semiMajorAxis = 0;
    float eccentricity = 0;
    float inclination = 0;
    float rightAscensionAscendingNode = 0;
    float argPeriapsis = 0;
    float trueAnomaly = 0;
    double radiusMagnitude = 0;
    double alpha = 0;
    double radiusPeriapsis = 0;
    double radiusApoapsis = 0;
};

class OrbitalMotion {
   public:
    static float eccentricToTrueAnomalyF32(float E, float e);
    static float eccentricToMeanAnomalyF32(float E, float e);
    static float trueToEccentricAnomalyF32(float f, float e);
    static float trueToHyperbolicAnomalyF32(float f, float e);
    static float trueToMeanAnomalyF32(float f, float e);
    static float hyperbolicToTrueAnomalyF32(float H, float e);
    static float hyperbolicToMeanAnomalyF32(float H, float e);
    static float meanToEccentricAnomalyF32(float M, float e);
    static float meanToTrueAnomalyF32(float M, float e);
    static float meanToHyperbolicAnomalyF32(float N, float e);
    static CartesianState elementsToCartesianStateF32(double mu, const ClassicalElementsF32& elements);
    static ClassicalElementsF32 cartesianStateToElementsF32(double mu,
                                                            const Eigen::Vector3d& rVec,
                                                            const Eigen::Vector3d& vVec);
};

#endif
