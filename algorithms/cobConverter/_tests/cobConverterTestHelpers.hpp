#ifndef TEST_COBCONVERTER_HELPERS_H
#define TEST_COBCONVERTER_HELPERS_H

#include "cobConverterAlgorithm.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/rigidBodyKinematics.hpp"
#include "utilities/fsw/safeMath.h"
#include "utilities/fsw/timeConstants.h"
#include <gtest/gtest.h>
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

inline Eigen::Vector3d mapState(const Eigen::Vector2d& pixel,
                                const Eigen::Matrix3d& cameraCalibrationMatrix,
                                const CalibrationCoefficients& coefficients) {
    const Eigen::Vector3d homogeneous(pixel(0), pixel(1), 1.0);
    const Eigen::Vector3d raw = cameraCalibrationMatrix.inverse() * homogeneous;
    const Eigen::Vector3d calibrated = applyBrownConrady(raw, coefficients);
    return -calibrated.normalized();
}

inline Eigen::Matrix3d mapCobCovar(double pixels, double dX, double dY) {
    const double X = 1.0 / dX;
    const double Y = 1.0 / dY;
    const double scaleFactor = safeSqrt(pixels / (4.0 * std::numbers::pi));
    Eigen::Matrix3d covar = Eigen::Matrix3d::Zero();
    covar(0, 0) = X * X;
    covar(1, 1) = Y * Y;
    covar(2, 2) = 1.0;
    return scaleFactor * covar;
}

inline Eigen::Matrix3d mapComCovar(double pixels,
                                   double fieldOfViewX,
                                   double fieldOfViewY,
                                   double resolutionX,
                                   double resolutionY,
                                   double dX,
                                   double dY,
                                   const Eigen::Vector3d& position,
                                   double radius,
                                   double alpha,
                                   const Eigen::Vector3d& sunUnit_N,
                                   double radiusUncertainty,
                                   double phi,
                                   const Eigen::Matrix3d& positionCovar) {
    const double X = 1.0 / dX;
    const double Y = 1.0 / dY;
    const double ifovX = fieldOfViewX / resolutionX;
    const double ifovY = fieldOfViewY / resolutionY;
    const double scaleFactor = safeSqrt(pixels / (4.0 * std::numbers::pi));

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
    const double sigmaBetaSquared = totalDeltaBinaryPartials + (deltaBinaryDeltaRadius * deltaBinaryDeltaRadius *
                                                                radiusUncertainty * radiusUncertainty);

    // sigmaBetaSquared is the variance of a 1-D magnitude along the sun direction (cos(phi),
    // sin(phi)); rotating it into image x/y via R(phi)*diag(sigmaBetaSquared,0)*R(phi)^T keeps the
    // result PSD by construction. Converting rad^2 -> normalized image-plane units needs both the
    // angle->pixel scale (ifovX/ifovY, average-scale) and the pixel->NIC scale (X/Y, exact tan-based)
    // -- they differ by up to ~26% at wide FOV, so both are applied via the congruence transform
    // D*(...)*D with D = diag(X/ifovX, Y/ifovY), which also preserves PSD-ness.
    const double cosPhi = safeCos(phi);
    const double sinPhi = safeSin(phi);
    const double directionX = X / ifovX;
    const double directionY = Y / ifovY;
    const double correctionXX = sigmaBetaSquared * directionX * directionX * cosPhi * cosPhi;
    const double correctionYY = sigmaBetaSquared * directionY * directionY * sinPhi * sinPhi;
    const double correctionXY = sigmaBetaSquared * directionX * directionY * cosPhi * sinPhi;

    Eigen::Matrix3d covarCom = Eigen::Matrix3d::Zero();
    covarCom(0, 0) = (X * X) + correctionXX;
    covarCom(1, 1) = (Y * Y) + correctionYY;
    covarCom(0, 1) = correctionXY;
    covarCom(1, 0) = correctionXY;
    covarCom(2, 2) = 1.0;
    return scaleFactor * covarCom;
}

}  // namespace cobConverterReference

// Mirrors CobConverterAlgorithm::updateState field-for-field.
inline CobConverterOutput referenceCobConverterUpdate(const CobConverterConfig& cfg,
                                                      const CobMeasurement& cob,
                                                      const VehicleAttitude& attitude,
                                                      const FilterState& filter) {
    CobConverterOutput output;

    if (!cob.cobValid || cob.cobPixelsFound == 0 ||
        filter.filterVehPosition.norm() <= static_cast<double>(cfg.getRadius())) {
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

    // alpha/phi/Rc/gamma (and validCOM) are only computed for a configured correction method; for
    // NoCorrectionAlg they stay zero, and gamma == 0 collapses COM onto COB regardless of Rc/phi.
    double gamma = 0.0;
    double phi = 0.0;
    double alpha = 0.0;
    double objectRadiusPixels = 0.0;
    Eigen::Vector3d shat_N = Eigen::Vector3d::Zero();

    const bool correctionRequested =
        cfg.getPhaseAngleCorrectionMethod() != PhaseAngleCorrectionMethodAlgorithm::NoCorrectionAlg;
    if (correctionRequested) {
        const Eigen::Vector3d position = filter.filterVehPosition;
        const Eigen::Vector3d rHat_N = position.normalized();
        const Eigen::Vector3d shat_B = attitude.vehSunPntBdy.cast<double>().normalized();
        shat_N = dcm_BN.transpose() * shat_B;
        const Eigen::Vector3d shat_C = dcm_CB * shat_B;

        alpha = safeAcos(rHat_N.dot(shat_N));
        phi = safeAtan2(shat_C(1), shat_C(0));

        if (cfg.getPhaseAngleCorrectionMethod() == PhaseAngleCorrectionMethodAlgorithm::BinaryAlg) {
            gamma = (4.0 / (3.0 * std::numbers::pi)) * (1.0 - safeCos(alpha));
        }
        objectRadiusPixels = static_cast<double>(cfg.getRadius()) * dX / position.norm();
    }

    const Eigen::Vector2d cobPixels = cob.cobCenterOfBrightness.cast<double>();
    const Eigen::Vector2d comPixels(cobPixels(0) - (gamma * objectRadiusPixels * safeCos(phi)),
                                    cobPixels(1) - (gamma * objectRadiusPixels * safeSin(phi)));
    // Mirrors CobConverterAlgorithm::updateState: validCom means "the resulting COM pixel location
    // is finite," applied the same way whether or not a correction was requested.
    const bool validCom = comPixels.allFinite();

    const CalibrationCoefficients coefficients = cfg.getCalibrationCoefficients();
    const Eigen::Vector3d rhatCOB_C = mapState(cobPixels, cameraCalibrationMatrix, coefficients);
    const Eigen::Vector3d rhatCOM_C = mapState(comPixels, cameraCalibrationMatrix, coefficients);

    const double pixelsFound = static_cast<double>(cob.cobPixelsFound);
    const Eigen::Matrix3d attitudeCovariance = cfg.getAttitudeCovariance().cast<double>();
    Eigen::Matrix3d covar_B;
    if (correctionRequested && cfg.getPhaseAngleCorrectionMethod() == PhaseAngleCorrectionMethodAlgorithm::BinaryAlg &&
        cfg.getRadiusUncertainty() > 0.0F) {
        const Eigen::Matrix3d covarCom_C = mapComCovar(pixelsFound,
                                                       fieldOfViewX,
                                                       fieldOfViewY,
                                                       resolutionX,
                                                       resolutionY,
                                                       dX,
                                                       dY,
                                                       filter.filterVehPosition,
                                                       static_cast<double>(cfg.getRadius()),
                                                       alpha,
                                                       shat_N,
                                                       static_cast<double>(cfg.getRadiusUncertainty()),
                                                       phi,
                                                       filter.filterVehPositionCovariance);
        const Eigen::Matrix3d covarCom_B = dcm_CB.transpose() * covarCom_C * dcm_CB;
        covar_B = covarCom_B + attitudeCovariance;
    } else {
        const Eigen::Matrix3d covarCob_C = mapCobCovar(pixelsFound, dX, dY);
        const Eigen::Matrix3d covarCob_B = dcm_CB.transpose() * covarCob_C * dcm_CB;
        covar_B = covarCob_B + attitudeCovariance;
    }

    bool coberrorOutlierTrigger = false;
    if (cfg.isOutlierDetectionEnabled()) {
        const Eigen::Vector3d rNav_N = filter.filterVehPosition;
        const Eigen::Vector3d rHatNav_N = rNav_N.normalized();
        const Eigen::Matrix3d covarNav_N = filter.filterVehPositionCovariance / (rNav_N.norm() * rNav_N.norm());

        Eigen::Vector3d rhatCOB_C_znorm = -rhatCOB_C;
        rhatCOB_C_znorm /= rhatCOB_C_znorm(2);
        const Eigen::Vector3d cob = cameraCalibrationMatrix * rhatCOB_C_znorm;

        Eigen::Vector3d rhatNav_C = dcm_NC.transpose() * (-rHatNav_N);
        rhatNav_C /= rhatNav_C(2);
        const Eigen::Vector3d cobNav = cameraCalibrationMatrix * rhatNav_C;

        const double cobErrorPrediction = (cob - cobNav).norm();

        double sigma = 0.0;
        if (cfg.isStandardDeviationSpecified()) {
            sigma = static_cast<double>(cfg.getStandardDeviation());
        } else {
            // Matches cobOutlierDetection's call into computeTotalCobCovariance, including
            // passing dcm_CB^T * covar_B * dcm_CB^T (not dcm_CB * covar_B * dcm_CB^T) as
            // "covarCob_C" -- see the namespace comment above.
            const Eigen::Matrix3d covarAtt_C = dcm_CB * attitudeCovariance * dcm_CB.transpose();
            const Eigen::Matrix3d dcm_CN = dcm_NC.transpose();
            const Eigen::Matrix3d covarNav_C = dcm_CN * covarNav_N * dcm_CN.transpose();
            const Eigen::Matrix3d covarCob_C_arg = dcm_CB.transpose() * covar_B * dcm_CB.transpose();
            const Eigen::Matrix3d covarTotal_C = covarCob_C_arg + covarAtt_C + covarNav_C;
            const Eigen::Matrix3d covarImage =
                cameraCalibrationMatrix * covarTotal_C * cameraCalibrationMatrix.transpose();
            sigma = safeSqrt(std::max(covarImage(0, 0), covarImage(1, 1)));
        }

        coberrorOutlierTrigger = !(cobErrorPrediction < static_cast<double>(cfg.getNumStandardDeviations()) * sigma);
    }

    const bool goodOutlierCheck = !coberrorOutlierTrigger;

    const Eigen::Vector3d rhatCOM_N = dcm_NC * rhatCOM_C;
    const Eigen::Vector3d rhatCOM_B = dcm_BN * rhatCOM_N;
    const Eigen::Matrix3d covar_N = dcm_BN.transpose() * covar_B * dcm_BN;
    const Eigen::Matrix3d covar_C = dcm_NC.transpose() * covar_N * dcm_NC;

    output.unitVec.covar_N = covar_N.cast<float>();
    output.unitVec.covar_C = covar_C.cast<float>();
    output.unitVec.covar_B = covar_B.cast<float>();
    output.unitVec.rhat_BN_N = rhatCOM_N.cast<float>();
    output.unitVec.rhat_BN_C = rhatCOM_C.cast<float>();
    output.unitVec.rhat_BN_B = rhatCOM_B.cast<float>();
    output.unitVec.unitVecTimeTag = static_cast<double>(cob.cobTimeTag) * kNano2Sec;
    // Mirrors CobConverterAlgorithm::populateOutputMessages: valid when the COM pixel location is
    // finite (validCom) and outlier detection didn't flag this cycle.
    output.unitVec.unitVecValid = validCom && goodOutlierCheck;

    output.com.centerOfBrightness = cobPixels.cast<float>();
    output.com.centerOfMass = comPixels.cast<float>();
    output.com.offsetFactor = static_cast<float>(gamma);
    output.com.objectPixelRadius = static_cast<int>(objectRadiusPixels);
    output.com.phaseAngle = static_cast<float>(alpha);
    output.com.sunDirection = static_cast<float>(phi);
    output.com.comTimeTag = cob.cobTimeTag;
    output.com.comValid = validCom;

    output.diagnostic.coberrorOutlierTrigger = coberrorOutlierTrigger;

    return output;
}

// The algorithm is FP32 and the reference above is double, so they never match exactly. Tolerances
// below are derived from operation count * float epsilon (1.19e-7) * margin, not picked by trial
// and error -- a wrong sign or dropped term is orders of magnitude bigger than any bound here.
//
// `tol` (1e-3F) covers rhat_BN_N/C/B, offsetFactor, phaseAngle, sunDirection. phaseAngle sets it:
// its acos(dot(rHat_N, shat_N)) is ill-conditioned as alpha -> 0 or pi (d(acos)/dx = -1/sin(alpha)),
// giving a floor of ~sqrt(2*n*epsilon) ~= 1.5e-3 for n ~ 5-10 upstream ops -- not a bug, since
// alpha ~= 0 (sun nearly behind the spacecraft) is a valid but ill-conditioned geometry. Confirmed
// empirically: tightening to 2e-4F failed on an alpha ~= 2.28e-4 rad case (2.6e-4 diff); reverted to
// 1e-3F. The other fields (~150-250 ops each: DCM builds, calibration, Brown-Conrady) only need
// ~2.4e-5 and share this bound with margin to spare.
//
// covar_N/C/B scale with radius * dX / range and (BinaryAlg) radiusUncertainty^2, from O(1) to
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
// a noise-scale floor alongside actual/reference.
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

inline void expectOutputsNear(const CobConverterOutput& out, const CobConverterOutput& ref, float tol) {
    constexpr float covarAtol = 1e-3F;
    constexpr float covarRtol = 1e-4F;
    const float covarNScale = covarNoiseScale(out.unitVec.covar_N, ref.unitVec.covar_N);
    const float covarCScale = covarNoiseScale(out.unitVec.covar_C, ref.unitVec.covar_C);
    const float covarBScale = covarNoiseScale(out.unitVec.covar_B, ref.unitVec.covar_B);
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(out.unitVec.rhat_BN_N(i), ref.unitVec.rhat_BN_N(i), tol);
        EXPECT_NEAR(out.unitVec.rhat_BN_C(i), ref.unitVec.rhat_BN_C(i), tol);
        EXPECT_NEAR(out.unitVec.rhat_BN_B(i), ref.unitVec.rhat_BN_B(i), tol);
        for (int j = 0; j < 3; ++j) {
            expectNear(out.unitVec.covar_N(i, j), ref.unitVec.covar_N(i, j), covarAtol, covarRtol, covarNScale);
            expectNear(out.unitVec.covar_C(i, j), ref.unitVec.covar_C(i, j), covarAtol, covarRtol, covarCScale);
            expectNear(out.unitVec.covar_B(i, j), ref.unitVec.covar_B(i, j), covarAtol, covarRtol, covarBScale);
        }
    }
    // finiteness
    EXPECT_TRUE(out.unitVec.rhat_BN_N.allFinite());
    EXPECT_TRUE(out.unitVec.rhat_BN_C.allFinite());
    EXPECT_TRUE(out.unitVec.rhat_BN_B.allFinite());
    EXPECT_TRUE(out.unitVec.covar_N.allFinite());
    EXPECT_TRUE(out.unitVec.covar_C.allFinite());
    EXPECT_TRUE(out.unitVec.covar_B.allFinite());

    EXPECT_NEAR(out.unitVec.unitVecTimeTag, ref.unitVec.unitVecTimeTag, 1e-9);
    EXPECT_EQ(out.unitVec.unitVecValid, ref.unitVec.unitVecValid);

    const float pixelNoiseScale =
        static_cast<float>(std::max(std::abs(out.com.objectPixelRadius), std::abs(ref.com.objectPixelRadius)));
    expectPixelNear(out.com.centerOfBrightness(0), ref.com.centerOfBrightness(0), pixelNoiseScale);
    expectPixelNear(out.com.centerOfBrightness(1), ref.com.centerOfBrightness(1), pixelNoiseScale);
    expectPixelNear(out.com.centerOfMass(0), ref.com.centerOfMass(0), pixelNoiseScale);
    expectPixelNear(out.com.centerOfMass(1), ref.com.centerOfMass(1), pixelNoiseScale);
    // finiteness
    EXPECT_TRUE(out.com.centerOfBrightness.allFinite());
    EXPECT_TRUE(out.com.centerOfMass.allFinite());

    expectPixelNear(
        static_cast<float>(out.com.objectPixelRadius), static_cast<float>(ref.com.objectPixelRadius), pixelNoiseScale);
    EXPECT_NEAR(out.com.offsetFactor, ref.com.offsetFactor, tol);
    expectAngleNear(out.com.phaseAngle, ref.com.phaseAngle, tol);
    expectAngleNear(out.com.sunDirection, ref.com.sunDirection, tol);
    // finiteness
    EXPECT_TRUE(std::isfinite(out.com.objectPixelRadius));
    EXPECT_TRUE(std::isfinite(out.com.offsetFactor));
    EXPECT_TRUE(std::isfinite(out.com.phaseAngle));
    EXPECT_TRUE(std::isfinite(out.com.sunDirection));

    EXPECT_EQ(out.com.comTimeTag, ref.com.comTimeTag);
    EXPECT_EQ(out.com.comValid, ref.com.comValid);
    EXPECT_EQ(out.diagnostic.coberrorOutlierTrigger, ref.diagnostic.coberrorOutlierTrigger);
}

// Takes raw config/input fields rather than a pre-built CobConverterConfig so this can later be
// fuzzed directly (FUZZ_TEST domains generate primitives, not validated objects -- and
// CobConverterConfig can only be obtained through the validating create()). A field combination
// that create() rejects isn't an algorithm bug, so it's skipped rather than failing the test;
// testCobConverterSetup() already covers validation itself.
inline void testCobConverter(PhaseAngleCorrectionMethodAlgorithm phaseAngleCorrectionMethod,
                             float radius,
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
        cfg = CobConverterConfig::create(phaseAngleCorrectionMethod,
                                         radius,
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
    CobConverterOutput out;
    EXPECT_NO_THROW(out = alg.updateState(cob, attitude, filter));
    const CobConverterOutput ref = referenceCobConverterUpdate(*cfg, cob, attitude, filter);

    // Always check validity agrees with the reference
    EXPECT_EQ(out.unitVec.unitVecValid, ref.unitVec.unitVecValid);
    EXPECT_EQ(out.com.comValid, ref.com.comValid);

    if (out.unitVec.unitVecValid && ref.unitVec.unitVecValid) {
        // See the tolerance comment above expectNear/expectOutputsNear.
        constexpr float fixedRangeTol = 1e-3F;
        expectOutputsNear(out, ref, fixedRangeTol);
    }
}

#endif  // TEST_COBCONVERTER_HELPERS_H
