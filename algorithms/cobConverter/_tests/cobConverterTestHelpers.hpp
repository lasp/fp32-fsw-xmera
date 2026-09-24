#ifndef TEST_COBCONVERTER_HELPERS_H
#define TEST_COBCONVERTER_HELPERS_H

#include "cobConverterAlgorithm.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/rigidBodyKinematics.hpp"
#include "utilities/fsw/safeMath.h"
#include "utilities/fsw/timeConstants.h"
#include <gtest/gtest.h>
#include <limits>
#include <numbers>
#include <optional>

// Double-precision reference, transcribed from cobConverterAlgorithm.cpp.
namespace cobConverterReference {

inline Eigen::Matrix3d computeCameraCalibrationMatrix(double fieldOfViewX,
                                                      double fieldOfViewY,
                                                      double resolutionX,
                                                      double resolutionY) {
    constexpr double alpha = 0.0;
    // safeTan (not std::tan) matches CobConverterAlgorithm::computeCameraParameters, which clamps
    // via safeTanf near the +/-pi/2 singularity. Using raw std::tan here would let the reference
    // diverge sharply from the algorithm's railed value whenever fieldOfViewX/fieldOfViewY combine
    // to push either argument close to the singularity, producing a spurious mismatch that
    // reflects this helper's precision choice rather than an algorithm defect.
    const double pX = 2.0 * safeTan(fieldOfViewX / 2.0);
    const double pY = 2.0 * safeTan(fieldOfViewY / 2.0);
    const double dX = resolutionX / pX;
    const double dY = resolutionY / pY;
    const double up = resolutionX / 2.0;
    const double vp = resolutionY / 2.0;
    Eigen::Matrix3d cameraCalibrationMatrix;
    cameraCalibrationMatrix << dX, alpha, up, 0.0, dY, vp, 0.0, 0.0, 1.0;
    return cameraCalibrationMatrix;
}

inline Eigen::Vector3d applyBrownConrady(const Eigen::Vector3d& uncalibratedVector,
                                         const CalibrationCoefficients& coefficients) {
    const double x = uncalibratedVector(0);
    const double y = uncalibratedVector(1);
    const double r2 = (x * x) + (y * y);
    const double r4 = r2 * r2;
    const double r6 = r2 * r4;
    const double kPolynomial = 1.0 + (coefficients.k1 * r2) + (coefficients.k2 * r4) + (coefficients.k3 * r6);
    Eigen::Vector3d calibratedVector;
    calibratedVector(0) =
        (x * kPolynomial) + (2.0 * coefficients.p1 * x * y) + (coefficients.p2 * (r2 + (2.0 * x * x)));
    calibratedVector(1) =
        (y * kPolynomial) + (2.0 * coefficients.p2 * x * y) + (coefficients.p1 * (r2 + (2.0 * y * y)));
    calibratedVector(2) = 1.0;
    return calibratedVector;
}

// Result of the double-precision Brown-Conrady inverse: undistorted homogeneous coordinate and
// whether the fixed-point iteration converged.
struct ReferenceUndistortion {
    Eigen::Vector3d vector;
    bool valid;
};

// Double-precision replica of CobConverterAlgorithm::undistortNormalizedCoordinate: the same
// fixed-point update, residual-based stop, iteration cap and guards, only evaluated in double.
inline ReferenceUndistortion undistortBrownConrady(const Eigen::Vector3d& distortedVector,
                                                   const CalibrationCoefficients& coefficients) {
    constexpr int kMaxIterations = 50;
    constexpr double kResidualTolerance = 1e-6;
    const double k1 = coefficients.k1;
    const double k2 = coefficients.k2;
    const double k3 = coefficients.k3;
    const double p1 = coefficients.p1;
    const double p2 = coefficients.p2;
    const double xDistorted = distortedVector(0);
    const double yDistorted = distortedVector(1);
    if (!std::isfinite(xDistorted) || !std::isfinite(yDistorted)) {
        return {{xDistorted, yDistorted, 1.0}, false};
    }
    double x = xDistorted;
    double y = yDistorted;
    for (int iteration = 0; iteration <= kMaxIterations; ++iteration) {
        const double r2 = (x * x) + (y * y);
        const double kPolynomial = 1.0 + (k1 * r2) + (k2 * r2 * r2) + (k3 * r2 * r2 * r2);
        const double deltaX = (2.0 * p1 * x * y) + (p2 * (r2 + (2.0 * x * x)));
        const double deltaY = (p1 * (r2 + (2.0 * y * y))) + (2.0 * p2 * x * y);
        const double residualX = (x * kPolynomial) + deltaX - xDistorted;
        const double residualY = (y * kPolynomial) + deltaY - yDistorted;
        const double residualScale = std::max(
            {1.0, std::abs(xDistorted), std::abs(yDistorted), std::abs(x * kPolynomial), std::abs(y * kPolynomial)});
        if (std::isfinite(residualScale) &&
            std::max(std::abs(residualX), std::abs(residualY)) <= kResidualTolerance * residualScale) {
            return {{x, y, 1.0}, true};
        }
        if (iteration == kMaxIterations ||
            std::abs(kPolynomial) < static_cast<double>(std::numeric_limits<float>::epsilon())) {
            break;
        }
        const double xNext = (xDistorted - deltaX) / kPolynomial;
        const double yNext = (yDistorted - deltaY) / kPolynomial;
        if (!std::isfinite(xNext) || !std::isfinite(yNext)) {
            break;
        }
        x = xNext;
        y = yNext;
    }
    return {{x, y, 1.0}, false};
}

inline Eigen::Vector3d mapState(const Eigen::Vector2d& pixel,
                                const Eigen::Matrix3d& cameraCalibrationMatrix,
                                const CalibrationCoefficients& coefficients,
                                bool* converged = nullptr) {
    const Eigen::Vector3d homogeneous(pixel(0), pixel(1), 1.0);
    const Eigen::Vector3d raw = cameraCalibrationMatrix.inverse() * homogeneous;
    const ReferenceUndistortion undistorted = undistortBrownConrady(raw, coefficients);
    if (converged != nullptr) {
        *converged = undistorted.valid;
    }
    return -undistorted.vector.normalized();
}

// Double-precision mirror of CobConverterAlgorithm::computeBetaVar.
inline double betaVariance(const Eigen::Vector3d& position,
                           double radius,
                           double alpha,
                           const Eigen::Vector3d& sunUnit_N,
                           double radiusUncertainty,
                           const Eigen::Matrix3d& positionCovar) {
    const double positionNorm = position.norm();
    const double oneMinusCosAlpha = 1.0 - safeCos(alpha);
    const double binaryTerm = (4.0 * radius / (3.0 * std::numbers::pi * positionNorm)) * oneMinusCosAlpha;
    const double constantsDeltaR = binaryTerm / (1.0 + (binaryTerm * binaryTerm));

    const Eigen::Vector3d rHat = position / positionNorm;
    const Eigen::RowVector3d deltaBinaryDeltaR = (-rHat / positionNorm * constantsDeltaR).transpose();
    const double deltaBinaryDeltaRadius = constantsDeltaR / radius;
    const double deltaBinaryDeltaAlphaCoeff =
        (4.0 * radius / (3.0 * std::numbers::pi * positionNorm)) / (1.0 + (binaryTerm * binaryTerm));

    // deltaAlphaDeltaR is d(alpha)/d(r) = -sunUnit_N^T/(r*sin(alpha)) * (I - rHat*rHat^T) (see
    // cobConverter.rst), but omits sin(alpha) since deltaBinaryDeltaAlphaCoeff above already omits
    // the matching factor and the two are only ever multiplied together below -- sin(alpha) cancels,
    // so only the sign matters here. Avoids a literal 1/sin(alpha) that blows up near alpha=0/pi.
    const Eigen::RowVector3d deltaAlphaDeltaR =
        -((sunUnit_N / positionNorm).transpose() * (Eigen::Matrix3d::Identity() - (rHat * rHat.transpose())));

    const Eigen::RowVector3d deltaBinaryR = deltaBinaryDeltaR + (deltaBinaryDeltaAlphaCoeff * deltaAlphaDeltaR);
    const double totalDeltaBinaryPartials = (deltaBinaryR * positionCovar * deltaBinaryR.transpose())(0, 0);
    return totalDeltaBinaryPartials +
           (deltaBinaryDeltaRadius * deltaBinaryDeltaRadius * radiusUncertainty * radiusUncertainty);
}

}  // namespace cobConverterReference

// Mirrors CobConverterAlgorithm::updateState field-for-field.
inline CobConverterUpdateResult referenceCobConverterUpdate(const CobConverterConfig& cfg,
                                                            const CobMeasurement& cob,
                                                            const VehicleAttitude& attitude,
                                                            const FilterState& filter,
                                                            bool* brownConradyConverged = nullptr,
                                                            bool* outlierNearThreshold = nullptr) {
    CobConverterUpdateResult output;

    if (!cob.cobValid || cob.cobPixelsFound == 0 ||
        filter.filterVehPosition.norm() <= static_cast<double>(cfg.getRadius()) || !attitude.vehSunPntBdy.allFinite() ||
        attitude.vehSunPntBdy.norm() == 0.0F) {
        return output;
    }

    using namespace cobConverterReference;

    const Eigen::Vector3d bodyToCameraMrp = cfg.getBodyToCameraMrp().cast<double>();
    const Eigen::Vector3d sigma_BN = attitude.sigma_BN.cast<double>();
    const Eigen::Matrix3d dcm_CB = mrpToDcm(bodyToCameraMrp);
    const Eigen::Matrix3d dcm_BN = mrpToDcm(sigma_BN);
    const Eigen::Matrix3d dcm_NC = (dcm_CB * dcm_BN).transpose();

    const double fieldOfViewX = static_cast<double>(cfg.getFieldOfViewX());
    const double fieldOfViewY = static_cast<double>(cfg.getFieldOfViewY());
    const double resolutionX = static_cast<double>(cfg.getResolutionX());
    const double resolutionY = static_cast<double>(cfg.getResolutionY());
    const Eigen::Matrix3d cameraCalibrationMatrix =
        computeCameraCalibrationMatrix(fieldOfViewX, fieldOfViewY, resolutionX, resolutionY);
    const double dX = cameraCalibrationMatrix(0, 0);
    const double dY = cameraCalibrationMatrix(1, 1);

    // The Binary correction is unconditional, so alpha/phi/Rc/gamma always compute.
    const Eigen::Vector3d position = filter.filterVehPosition;
    const Eigen::Vector3d rHat_N = position.normalized();
    const Eigen::Vector3d shat_B = attitude.vehSunPntBdy.cast<double>().normalized();
    const Eigen::Vector3d shat_N = dcm_BN.transpose() * shat_B;
    const Eigen::Vector3d shat_C = dcm_CB * shat_B;

    const double alpha = safeAcos(rHat_N.dot(shat_N));
    const double phi = safeAtan2(shat_C(1), shat_C(0));
    const double gamma = (4.0 / (3.0 * std::numbers::pi)) * (1.0 - safeCos(alpha));
    const double objectRadiusPixels = static_cast<double>(cfg.getRadius()) * dX / position.norm();
    const double tanBeta = static_cast<double>(cfg.getRadius()) * gamma / position.norm();

    const Eigen::Vector2d cobPixels = cob.cobCenterOfBrightness.cast<double>();
    const Eigen::Vector2d comPixels(cobPixels(0) - (tanBeta * dX * safeCos(phi)),
                                    cobPixels(1) - (tanBeta * dY * safeSin(phi)));
    // Mirrors CobConverterAlgorithm::updateState: validCom means "the resulting COM pixel location
    // is finite," applied the same way whether or not a correction was requested.
    const bool validCom = comPixels.allFinite();

    const CalibrationCoefficients coefficients = cfg.getCalibrationCoefficients();
    bool cobConverged = false;
    bool comConverged = false;
    const Eigen::Vector3d rhatCOB_C = mapState(cobPixels, cameraCalibrationMatrix, coefficients, &cobConverged);
    const Eigen::Vector3d rhatCOM_C = mapState(comPixels, cameraCalibrationMatrix, coefficients, &comConverged);
    if (brownConradyConverged != nullptr) {
        *brownConradyConverged = cobConverged && comConverged;
    }

    // Mirrors updateState's COM unit-vector covariance.
    const double pixelsFound = static_cast<double>(cob.cobPixelsFound);
    const Eigen::Matrix3d attitudeCovariance = cfg.getAttitudeCovariance().cast<double>();
    const Eigen::Matrix2d covarCOBuv = (pixelsFound / (4.0 * std::numbers::pi)) * Eigen::Matrix2d::Identity();
    const double onePlusTanBetaSq = 1.0 + (tanBeta * tanBeta);
    const double phaseVar = betaVariance(filter.filterVehPosition,
                                         static_cast<double>(cfg.getRadius()),
                                         alpha,
                                         shat_N,
                                         static_cast<double>(cfg.getRadiusUncertainty()),
                                         filter.filterVehPositionCovariance) *
                            onePlusTanBetaSq * onePlusTanBetaSq;
    Eigen::Matrix2d pixelToNormalized = Eigen::Matrix2d::Zero();
    pixelToNormalized(0, 0) = 1.0 / dX;
    pixelToNormalized(1, 1) = 1.0 / dY;
    const Eigen::Vector2d sunDirection(safeCos(phi), safeSin(phi));
    const Eigen::Matrix2d covarCOMxy = (pixelToNormalized * covarCOBuv * pixelToNormalized.transpose()) +
                                       (phaseVar * (sunDirection * sunDirection.transpose()));

    const Eigen::Vector3d h = rhatCOM_C / rhatCOM_C(2);  // undistorted COM [x, y, 1]
    const double x = h(0);
    const double y = h(1);
    Eigen::Matrix<double, 3, 2> jacobian;
    jacobian << (y * y) + 1.0, -x * y, -x * y, (x * x) + 1.0, -x, -y;
    jacobian *= -1.0 / std::pow(h.norm(), 3);
    const Eigen::Matrix3d covarRHat_C = jacobian * covarCOMxy * jacobian.transpose();

    const Eigen::Vector3d rHat_B = dcm_CB.transpose() * rhatCOM_C;
    Eigen::Matrix3d rHatSkew_B;
    rHatSkew_B << 0.0, -rHat_B(2), rHat_B(1), rHat_B(2), 0.0, -rHat_B(0), -rHat_B(1), rHat_B(0), 0.0;
    const Eigen::Matrix3d covar_N =
        (dcm_NC * covarRHat_C * dcm_NC.transpose()) +
        (16.0 * dcm_BN.transpose() * rHatSkew_B * attitudeCovariance * rHatSkew_B.transpose() * dcm_BN);
    const Eigen::Matrix3d covar_B = dcm_BN * covar_N * dcm_BN.transpose();

    // Mirrors updateState's publish gate: COM unit vector and covariance must be finite.
    if (!rhatCOM_C.allFinite() || !covar_N.allFinite()) {
        return output;
    }

    bool goodOutlierCheck = true;
    bool comErrorOutlierTrigger = false;
    if (cfg.isOutlierDetectionEnabled()) {
        // Mirrors comOutlierDetection: COM heading vs filter heading, both COM->SC in N.
        const Eigen::Vector3d rhatNav_N = filter.filterVehPosition.normalized();
        const double error = ((dcm_NC * rhatCOM_C) - rhatNav_N).norm();
        double sigma = 0.0;
        if (cfg.isStandardDeviationSpecified()) {
            sigma = static_cast<double>(cfg.getStandardDeviation()) * safeSqrt((1.0 / (dX * dX)) + (1.0 / (dY * dY)));
        } else {
            const Eigen::Matrix3d projection = Eigen::Matrix3d::Identity() - (rhatNav_N * rhatNav_N.transpose());
            const Eigen::Matrix3d covarNav_N = projection * filter.filterVehPositionCovariance *
                                               projection.transpose() / filter.filterVehPosition.squaredNorm();
            sigma = safeSqrt((covar_N + covarNav_N).trace());
        }
        const double threshold = static_cast<double>(cfg.getNumStandardDeviations()) * sigma;
        goodOutlierCheck = error < threshold;
        comErrorOutlierTrigger = !goodOutlierCheck;
        // fp32 and double may disagree within rounding noise of the gate: 1e-2 relative (sigma) + 1e-6 absolute
        // (chord).
        if (outlierNearThreshold != nullptr) {
            *outlierNearThreshold = std::abs(error - threshold) < (1e-2 * threshold) + 1e-6;
        }
    }

    const Eigen::Vector3d rhatCOM_N = dcm_NC * rhatCOM_C;
    const Eigen::Vector3d rhatCOM_B = dcm_BN * rhatCOM_N;
    const Eigen::Matrix3d covar_C = dcm_NC.transpose() * covar_N * dcm_NC;

    output.output.covar_N = covar_N.cast<float>();
    output.output.rhat_BN_N = rhatCOM_N.cast<float>();
    output.output.unitVecTimeTag = static_cast<double>(cob.cobTimeTag) * kNano2Sec;
    // Mirrors updateState: finite COM and no outlier flag.
    output.output.unitVecValid = validCom && goodOutlierCheck;

    output.diagnostic.covar_C = covar_C.cast<float>();
    output.diagnostic.covar_B = covar_B.cast<float>();
    output.diagnostic.rhat_BN_C = rhatCOM_C.cast<float>();
    output.diagnostic.rhat_BN_B = rhatCOM_B.cast<float>();
    output.diagnostic.rhat_COB_C = rhatCOB_C.cast<float>();
    output.diagnostic.rhat_COB_N = (dcm_NC * rhatCOB_C).cast<float>();
    output.diagnostic.centerOfBrightness = cobPixels.cast<float>();
    output.diagnostic.centerOfMass = comPixels.cast<float>();
    output.diagnostic.offsetFactor = static_cast<float>(gamma);
    output.diagnostic.objectPixelRadius = static_cast<int>(objectRadiusPixels);
    output.diagnostic.phaseAngle = static_cast<float>(alpha);
    output.diagnostic.sunDirection = static_cast<float>(phi);
    output.diagnostic.comTimeTag = cob.cobTimeTag;
    output.diagnostic.comValid = validCom;
    output.diagnostic.comErrorOutlierTrigger = comErrorOutlierTrigger;

    return output;
}

// The algorithm is FP32 and the reference above is double, so they never match exactly. Tolerances
// below are derived from operation count * float epsilon (1.19e-7) * margin, not picked by trial
// and error -- a wrong sign or dropped term is orders of magnitude bigger than any bound here.
//
// `tol` (1e-3F) covers rhat_BN_N/C/B, rhat_COB_C/N, offsetFactor, phaseAngle, sunDirection.
// phaseAngle sets it:
// its acos(dot(rHat_N, shat_N)) is ill-conditioned as alpha -> 0 or pi (d(acos)/dx = -1/sin(alpha)),
// giving a floor of ~sqrt(2*n*epsilon) ~= 1.5e-3 for n ~ 5-10 upstream ops -- not a bug, since
// alpha ~= 0 (sun nearly behind the spacecraft) is a valid but ill-conditioned geometry. Confirmed
// empirically: tightening to 2e-4F failed on an alpha ~= 2.28e-4 rad case (2.6e-4 diff); reverted to
// 1e-3F. The other fields (~150-250 ops each: DCM builds, calibration, Brown-Conrady) only need
// ~2.4e-5 and share this bound with margin to spare.
//
// covar_N/C/B scale with radius * dX / range and radiusUncertainty^2, from O(1) to
// O(1e6)+, where a single ULP exceeds 1e-3. Use atol + rtol*max(|reference|, noiseScale) instead:
//   - covarRtol = 1e-4F, covarAtol = 1e-3F: both sized off the pipeline's ~2.4e-5 baseline
//     (~150-250 ops). A prior radius>>range + anisotropic-covariance case pushed the observed error
//     to ~1.5e-4 via cancellation, but that geometry is now excluded by the radius>=range skip
//     below, so both hold clean over a 300s/22.7M-execution fuzz run.
//   - noiseScale (matrix diagonal magnitude): off-diagonal cross terms cancel to ~0 but still carry
//     rounding noise set by the matrix's overall scale, not their own near-zero value -- scaling
//     rtol by |reference| alone (as the xmera Python test still does) collapses back to the bare
//     atol in exactly that case.
//
// unitVecTimeTag gets a tight 1e-9: it's `cobTimeTag * kNano2Sec` in double on both sides, so
// there's no FP32 rounding to absorb -- just double round-off.
inline void expectNear(float actual, float reference, float atol, float rtol, float noiseScale) {
    EXPECT_NEAR(actual, reference, atol + (rtol * std::max(std::abs(reference), noiseScale)));
}

// centerOfBrightness/centerOfMass/objectPixelRadius are pixel coordinates whose magnitude scales
// with radius * dX / range, reaching O(1e5)+ for narrow fieldOfView + large radius. A flat +-1px
// bound covers most cases, but at that scale relative rounding drift exceeds 1px while staying
// physically meaningless, so add an rtol term scaled by magnitude: kPixelRtol = 1e-4F holds clean
// over the same 300s fuzz run as covarRtol above (the radius>=range skip below removes the
// pathological magnitudes that would otherwise need a looser bound).
//
// centerOfMass's correction term (gamma * objectRadiusPixels * cos(phi)/sin(phi)) can be large even
// when it nearly cancels cobCenterOfBrightness, landing the *reference* near zero -- scaling rtol by
// |reference| alone would collapse back to the bare 1px bound despite real rounding noise set by
// objectRadiusPixels' magnitude, not the cancelled result. Pass objectRadiusPixels in explicitly as
// a noise-scale floor alongside actual/reference. The COM y offset scales with dY, not dX, so its
// floor is rescaled by dY/dX.
constexpr float kPixelRtol = 1e-4F;
inline void expectPixelNear(float actual, float reference, float noiseScale) {
    EXPECT_LE(std::abs(actual - reference),
              1.0F + (kPixelRtol * std::max({std::abs(actual), std::abs(reference), noiseScale})));
}

// sunDirection (phi) comes from atan2, whose range wraps at +/-pi: two directions that are
// physically identical (or a hair apart) can print as e.g. +3.14159 and -3.14159 whenever ordinary
// FP32-vs-double rounding lands them on opposite sides of that branch cut, which a plain linear
// EXPECT_NEAR sees as a ~2*pi difference instead of a near-zero one. Compare the wrapped difference
// (the smallest signed angle between the two, in (-pi, pi]) instead of the raw one.
inline void expectAngleNear(float actual, float reference, float tol) {
    const float wrapped = std::remainder(actual - reference, 2.0F * std::numbers::pi_v<float>);
    EXPECT_LE(std::abs(wrapped), tol);
}

// The diagonal (variances, >= 0) stands in for the matrix's overall magnitude: off-diagonal cross
// terms can cancel to ~0 even when the diagonal is huge, but their rounding noise scales with the
// diagonal, not their own near-zero value.
inline float covarNoiseScale(const Eigen::Matrix3f& actual, const Eigen::Matrix3f& reference) {
    return std::max({std::abs(actual(0, 0)),
                     std::abs(actual(1, 1)),
                     std::abs(actual(2, 2)),
                     std::abs(reference(0, 0)),
                     std::abs(reference(1, 1)),
                     std::abs(reference(2, 2))});
}

inline void expectOutputsNear(const CobConverterUpdateResult& out,
                              const CobConverterUpdateResult& ref,
                              float tol,
                              float pixelScaleRatioY) {
    constexpr float covarAtol = 1e-3F;
    constexpr float covarRtol = 1e-4F;
    const float covarNScale = covarNoiseScale(out.output.covar_N, ref.output.covar_N);
    const float covarCScale = covarNoiseScale(out.diagnostic.covar_C, ref.diagnostic.covar_C);
    const float covarBScale = covarNoiseScale(out.diagnostic.covar_B, ref.diagnostic.covar_B);
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(out.output.rhat_BN_N(i), ref.output.rhat_BN_N(i), tol);
        EXPECT_NEAR(out.diagnostic.rhat_BN_C(i), ref.diagnostic.rhat_BN_C(i), tol);
        EXPECT_NEAR(out.diagnostic.rhat_BN_B(i), ref.diagnostic.rhat_BN_B(i), tol);
        EXPECT_NEAR(out.diagnostic.rhat_COB_C(i), ref.diagnostic.rhat_COB_C(i), tol);
        EXPECT_NEAR(out.diagnostic.rhat_COB_N(i), ref.diagnostic.rhat_COB_N(i), tol);
        for (int j = 0; j < 3; ++j) {
            expectNear(out.output.covar_N(i, j), ref.output.covar_N(i, j), covarAtol, covarRtol, covarNScale);
            expectNear(out.diagnostic.covar_C(i, j), ref.diagnostic.covar_C(i, j), covarAtol, covarRtol, covarCScale);
            expectNear(out.diagnostic.covar_B(i, j), ref.diagnostic.covar_B(i, j), covarAtol, covarRtol, covarBScale);
        }
    }
    // finiteness
    EXPECT_TRUE(out.output.rhat_BN_N.allFinite());
    EXPECT_TRUE(out.diagnostic.rhat_BN_C.allFinite());
    EXPECT_TRUE(out.diagnostic.rhat_BN_B.allFinite());
    EXPECT_TRUE(out.diagnostic.rhat_COB_C.allFinite());
    EXPECT_TRUE(out.diagnostic.rhat_COB_N.allFinite());
    EXPECT_TRUE(out.output.covar_N.allFinite());
    EXPECT_TRUE(out.diagnostic.covar_C.allFinite());
    EXPECT_TRUE(out.diagnostic.covar_B.allFinite());

    EXPECT_NEAR(out.output.unitVecTimeTag, ref.output.unitVecTimeTag, 1e-9);
    EXPECT_EQ(out.output.unitVecValid, ref.output.unitVecValid);

    const float pixelNoiseScale = static_cast<float>(
        std::max(std::abs(out.diagnostic.objectPixelRadius), std::abs(ref.diagnostic.objectPixelRadius)));
    expectPixelNear(out.diagnostic.centerOfBrightness(0), ref.diagnostic.centerOfBrightness(0), pixelNoiseScale);
    expectPixelNear(out.diagnostic.centerOfBrightness(1), ref.diagnostic.centerOfBrightness(1), pixelNoiseScale);
    expectPixelNear(out.diagnostic.centerOfMass(0), ref.diagnostic.centerOfMass(0), pixelNoiseScale);
    const float pixelNoiseScaleY = pixelNoiseScale * pixelScaleRatioY;
    expectPixelNear(out.diagnostic.centerOfMass(1), ref.diagnostic.centerOfMass(1), pixelNoiseScaleY);
    // finiteness
    EXPECT_TRUE(out.diagnostic.centerOfBrightness.allFinite());
    EXPECT_TRUE(out.diagnostic.centerOfMass.allFinite());

    expectPixelNear(static_cast<float>(out.diagnostic.objectPixelRadius),
                    static_cast<float>(ref.diagnostic.objectPixelRadius),
                    pixelNoiseScale);
    EXPECT_NEAR(out.diagnostic.offsetFactor, ref.diagnostic.offsetFactor, tol);
    expectAngleNear(out.diagnostic.phaseAngle, ref.diagnostic.phaseAngle, tol);
    expectAngleNear(out.diagnostic.sunDirection, ref.diagnostic.sunDirection, tol);
    // finiteness
    EXPECT_TRUE(std::isfinite(out.diagnostic.objectPixelRadius));
    EXPECT_TRUE(std::isfinite(out.diagnostic.offsetFactor));
    EXPECT_TRUE(std::isfinite(out.diagnostic.phaseAngle));
    EXPECT_TRUE(std::isfinite(out.diagnostic.sunDirection));

    EXPECT_EQ(out.diagnostic.comTimeTag, ref.diagnostic.comTimeTag);
    EXPECT_EQ(out.diagnostic.comValid, ref.diagnostic.comValid);
    EXPECT_EQ(out.diagnostic.comErrorOutlierTrigger, ref.diagnostic.comErrorOutlierTrigger);
}

// Takes raw config/input fields rather than a pre-built CobConverterConfig so this can later be
// fuzzed directly (FUZZ_TEST domains generate primitives, not validated objects -- and
// CobConverterConfig can only be obtained through the validating create()). A field combination
// that create() rejects isn't an algorithm bug, so it's skipped rather than failing the test;
// testCobConverterSetup() already covers validation itself.
inline void testCobConverter(float radius,
                             float radiusUncertainty,
                             const Eigen::Matrix3f& attitudeCovariance,
                             float numStandardDeviations,
                             float standardDeviation,
                             bool specifiedStandardDeviation,
                             bool outlierDetectionEnabled,
                             const CalibrationCoefficients& calibrationCoefficients,
                             int cameraId,
                             float fieldOfViewX,
                             float fieldOfViewY,
                             float resolutionX,
                             float resolutionY,
                             const Eigen::Vector3f& bodyToCameraMrp,
                             bool cobValid,
                             int32_t cobPixelsFound,
                             const Eigen::Vector2f& cobCenterOfBrightness,
                             uint64_t cobTimeTag,
                             const Eigen::Vector3f& sigma_BN,
                             const Eigen::Vector3f& vehSunPntBdy,
                             const Eigen::Vector3d& filterVehPosition,
                             const Eigen::Matrix3d& filterVehPositionCovariance) {
    std::optional<CobConverterConfig> cfg;
    try {
        cfg = CobConverterConfig::create(radius,
                                         radiusUncertainty,
                                         attitudeCovariance,
                                         numStandardDeviations,
                                         standardDeviation,
                                         specifiedStandardDeviation,
                                         outlierDetectionEnabled,
                                         calibrationCoefficients,
                                         cameraId,
                                         fieldOfViewX,
                                         fieldOfViewY,
                                         resolutionX,
                                         resolutionY,
                                         bodyToCameraMrp);
    } catch (const fsw::invalid_argument&) {
        return;
    }

    const CobMeasurement cob{.cobValid = cobValid,
                             .cobPixelsFound = cobPixelsFound,
                             .cobCenterOfBrightness = cobCenterOfBrightness,
                             .cobTimeTag = cobTimeTag};
    const VehicleAttitude attitude{.sigma_BN = sigma_BN, .vehSunPntBdy = vehSunPntBdy};
    const FilterState filter{.filterVehPosition = filterVehPosition,
                             .filterVehPositionCovariance = filterVehPositionCovariance};

    CobConverterAlgorithm alg(*cfg);
    CobConverterUpdateResult out;
    EXPECT_NO_THROW(out = alg.updateState(cob, attitude, filter));
    bool brownConradyConverged = true;
    bool outlierNearThreshold = false;
    const CobConverterUpdateResult ref =
        referenceCobConverterUpdate(*cfg, cob, attitude, filter, &brownConradyConverged, &outlierNearThreshold);

    // No well-defined answer: non-converged Brown-Conrady inverse (either precision) or error at the outlier gate.
    if (!brownConradyConverged || !out.diagnostic.brownConradyValid || outlierNearThreshold) {
        return;
    }

    // Always check validity agrees with the reference
    EXPECT_EQ(out.output.unitVecValid, ref.output.unitVecValid);
    EXPECT_EQ(out.diagnostic.comValid, ref.diagnostic.comValid);

    if (out.output.unitVecValid) {
        // See the tolerance comment above expectNear/expectOutputsNear.
        constexpr float fixedRangeTol = 1e-3F;
        const Eigen::Matrix3d cameraCalibrationMatrix =
            cobConverterReference::computeCameraCalibrationMatrix(static_cast<double>(cfg->getFieldOfViewX()),
                                                                  static_cast<double>(cfg->getFieldOfViewY()),
                                                                  static_cast<double>(cfg->getResolutionX()),
                                                                  static_cast<double>(cfg->getResolutionY()));
        const auto pixelScaleRatioY = static_cast<float>(cameraCalibrationMatrix(1, 1) / cameraCalibrationMatrix(0, 0));
        expectOutputsNear(out, ref, fixedRangeTol, pixelScaleRatioY);
    }
}

#endif  // TEST_COBCONVERTER_HELPERS_H
