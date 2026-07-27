#ifndef TEST_COBCONVERTER_HELPERS_H
#define TEST_COBCONVERTER_HELPERS_H

#include "cobConverterAlgorithm.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/rigidBodyKinematics.hpp"
#include "utilities/fsw/timeConstants.h"
#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <numbers>
#include <optional>

// Double-precision reference, transcribed from cobConverterAlgorithm.cpp.
namespace cobConverterReference {

inline Eigen::Matrix3d computeCameraCalibrationMatrix(double fieldOfView, double resolutionX, double resolutionY) {
    constexpr double alpha = 0.0;
    const double pX = 2.0 * std::tan(fieldOfView / 2.0);
    const double pY = 2.0 * std::tan(fieldOfView * resolutionY / resolutionX / 2.0);
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
    const double scaleFactor = std::sqrt(pixels / (4.0 * std::numbers::pi));
    Eigen::Matrix3d covar = Eigen::Matrix3d::Zero();
    covar(0, 0) = X * X;
    covar(1, 1) = Y * Y;
    covar(2, 2) = 1.0;
    return scaleFactor * covar;
}

inline Eigen::Matrix3d mapComCovar(double pixels,
                                   double fieldOfView,
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
    // ifovX = fieldOfView/dX*pX, and pX = resolutionX/dX (dX = resolutionX/pX), so this reduces
    // to fieldOfView*resolutionX/dX^2 -- caller already has dX/dY, no need to re-derive via tan().
    const double ifovX = fieldOfView * resolutionX / (dX * dX);
    const double ifovY = fieldOfView * resolutionY / (dY * dY);
    const double scaleFactor = std::sqrt(pixels / (4.0 * std::numbers::pi));

    const double positionNorm = position.norm();
    const double oneMinusCosAlpha = 1.0 - std::cos(alpha);
    const double binaryTerm = (4.0 * radius / (3.0 * std::numbers::pi * positionNorm)) * oneMinusCosAlpha;
    const double constantsDeltaR = binaryTerm / (1.0 + (binaryTerm * binaryTerm));

    const Eigen::Vector3d rHat = position / positionNorm;
    const Eigen::RowVector3d deltaBinaryDeltaR = (-rHat / positionNorm * constantsDeltaR).transpose();
    const double deltaBinaryDeltaRadius = constantsDeltaR / radius;
    const double deltaBinaryDeltaAlphaCoeff =
        (4.0 * radius / (3.0 * std::numbers::pi * positionNorm)) / (1.0 + (binaryTerm * binaryTerm));

    const Eigen::RowVector3d deltaAlphaDeltaR =
        (sunUnit_N / positionNorm).transpose() * (Eigen::Matrix3d::Identity() - (rHat * rHat.transpose()));

    const Eigen::RowVector3d deltaBinaryR = deltaBinaryDeltaR + (deltaBinaryDeltaAlphaCoeff * deltaAlphaDeltaR);
    const double totalDeltaBinaryPartials = (deltaBinaryR * positionCovar * deltaBinaryR.transpose())(0, 0);
    const double sigmaBetaSquared = totalDeltaBinaryPartials + (deltaBinaryDeltaRadius * deltaBinaryDeltaRadius *
                                                                radiusUncertainty * radiusUncertainty);

    Eigen::Matrix3d covarCom = Eigen::Matrix3d::Zero();
    covarCom(0, 0) = (X * X) + ((sigmaBetaSquared / (ifovX * ifovX)) * std::cos(phi));
    covarCom(1, 1) = (Y * Y) + ((sigmaBetaSquared / (ifovY * ifovY)) * std::sin(phi));
    covarCom(2, 2) = 1.0;
    return scaleFactor * covarCom;
}

}  // namespace cobConverterReference

// Mirrors CobConverterAlgorithm::updateState field-for-field.
inline CobConverterOutput referenceCobConverterUpdate(const CobConverterConfig& cfg, const CobConverterInput& input) {
    CobConverterOutput output;

    if (!input.cobValid || input.cobPixelsFound == 0) {
        return output;
    }

    using namespace cobConverterReference;

    const Eigen::Vector3d bodyToCameraMrp = cfg.getBodyToCameraMrp().cast<double>();
    const Eigen::Vector3d sigma_BN = input.sigma_BN.cast<double>();
    const Eigen::Matrix3d dcm_CB = mrpToDcm(bodyToCameraMrp);
    const Eigen::Matrix3d dcm_BN = mrpToDcm(sigma_BN);
    const Eigen::Matrix3d dcm_NC = (dcm_CB * dcm_BN).transpose();

    const double fieldOfView = static_cast<double>(cfg.getFieldOfView());
    const double resolutionX = static_cast<double>(cfg.getResolutionX());
    const double resolutionY = static_cast<double>(cfg.getResolutionY());
    const Eigen::Matrix3d cameraCalibrationMatrix =
        computeCameraCalibrationMatrix(fieldOfView, resolutionX, resolutionY);
    const double dX = cameraCalibrationMatrix(0, 0);
    const double dY = cameraCalibrationMatrix(1, 1);

    // alpha/phi/Rc/gamma (and validCOM) are only computed for a configured correction method; for
    // NoCorrectionAlg they stay zero, and gamma == 0 collapses COM onto COB regardless of Rc/phi.
    double gamma = 0.0;
    double phi = 0.0;
    double alpha = 0.0;
    double objectRadiusPixels = 0.0;
    Eigen::Vector3d shat_N = Eigen::Vector3d::Zero();
    bool validCom = false;

    const bool correctionRequested =
        cfg.getPhaseAngleCorrectionMethod() != PhaseAngleCorrectionMethodAlgorithm::NoCorrectionAlg;
    if (correctionRequested) {
        validCom = true;
        const Eigen::Vector3d position = input.filterVehPosition;
        const Eigen::Vector3d rHat_N = position.normalized();
        const Eigen::Vector3d shat_B = input.vehSunPntBdy.cast<double>().normalized();
        shat_N = dcm_BN.transpose() * shat_B;
        const Eigen::Vector3d shat_C = dcm_CB * shat_B;

        alpha = std::acos(rHat_N.dot(shat_N));
        phi = std::atan2(shat_C(1), shat_C(0));

        if (cfg.getPhaseAngleCorrectionMethod() == PhaseAngleCorrectionMethodAlgorithm::BinaryAlg) {
            gamma = (4.0 / (3.0 * std::numbers::pi)) * (1.0 - std::cos(alpha));
        }
        objectRadiusPixels = static_cast<double>(cfg.getRadius()) * dX / position.norm();
    }

    const Eigen::Vector2d cobPixels = input.cobCenterOfBrightness.cast<double>();
    const Eigen::Vector2d comPixels(cobPixels(0) - (gamma * objectRadiusPixels * std::cos(phi)),
                                    cobPixels(1) - (gamma * objectRadiusPixels * std::sin(phi)));

    const CalibrationCoefficients coefficients = cfg.getCalibrationCoefficients();
    const Eigen::Vector3d rhatCOB_C = mapState(cobPixels, cameraCalibrationMatrix, coefficients);
    const Eigen::Vector3d rhatCOM_C = mapState(comPixels, cameraCalibrationMatrix, coefficients);

    const double pixelsFound = static_cast<double>(input.cobPixelsFound);
    const Eigen::Matrix3d attitudeCovariance = cfg.getAttitudeCovariance().cast<double>();
    Eigen::Matrix3d covar_B;
    if (correctionRequested && cfg.getPhaseAngleCorrectionMethod() == PhaseAngleCorrectionMethodAlgorithm::BinaryAlg &&
        cfg.getRadiusUncertainty() > 0.0F) {
        const Eigen::Matrix3d covarCom_C = mapComCovar(pixelsFound,
                                                       fieldOfView,
                                                       resolutionX,
                                                       resolutionY,
                                                       dX,
                                                       dY,
                                                       input.filterVehPosition,
                                                       static_cast<double>(cfg.getRadius()),
                                                       alpha,
                                                       shat_N,
                                                       static_cast<double>(cfg.getRadiusUncertainty()),
                                                       phi,
                                                       input.filterVehPositionCovariance);
        const Eigen::Matrix3d covarCom_B = dcm_CB.transpose() * covarCom_C * dcm_CB;
        covar_B = covarCom_B + attitudeCovariance;
    } else {
        const Eigen::Matrix3d covarCob_C = mapCobCovar(pixelsFound, dX, dY);
        const Eigen::Matrix3d covarCob_B = dcm_CB.transpose() * covarCob_C * dcm_CB;
        covar_B = covarCob_B + attitudeCovariance;
    }

    bool coberrorOutlierTrigger = false;
    if (cfg.isOutlierDetectionEnabled()) {
        const Eigen::Vector3d rNav_N = input.filterVehPosition;
        const Eigen::Vector3d rHatNav_N = rNav_N.normalized();
        const Eigen::Matrix3d covarNav_N = input.filterVehPositionCovariance / (rNav_N.norm() * rNav_N.norm());

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
            sigma = std::sqrt(std::max(covarImage(0, 0), covarImage(1, 1)));
        }

        coberrorOutlierTrigger = !(cobErrorPrediction < static_cast<double>(cfg.getNumStandardDeviations()) * sigma);
    }

    // goodOutlierCheck defaults true and cobOutlierDetection never sets it false on a trigger, so
    // unitVecValid never actually reflects the outlier check. Matches current algorithm behavior.
    constexpr bool goodOutlierCheck = true;

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
    output.unitVec.unitVecTimeTag = static_cast<double>(input.cobTimeTag) * kNano2Sec;
    output.unitVec.unitVecValid = validCom && goodOutlierCheck;

    output.com.centerOfBrightness = cobPixels.cast<float>();
    output.com.centerOfMass = comPixels.cast<float>();
    output.com.offsetFactor = static_cast<float>(gamma);
    output.com.objectPixelRadius = static_cast<int>(objectRadiusPixels);
    output.com.phaseAngle = static_cast<float>(alpha);
    output.com.sunDirection = static_cast<float>(phi);
    output.com.comTimeTag = input.cobTimeTag;
    output.com.comValid = validCom;

    output.diagnostic.coberrorOutlierTrigger = coberrorOutlierTrigger;

    return output;
}

// The algorithm is FP32 and the reference above is double, so they never match exactly -- these
// tolerances just need to absorb that rounding drift without hiding an actual regression (a wrong
// sign or a dropped term is orders of magnitude bigger than rounding noise).
//
// `tol` (1e-3F, from testCobConverter) covers fields with a fixed, moderate range: unit vector
// components ([-1, 1]), pixel coordinates (O(1)-O(1e3)), and the phase-angle/sun-direction/
// offset-factor scalars (O(1)). FP32 rounding through the rotation/trig/distortion pipeline lands
// around 1e-5 to 1e-4 for these, so 1e-3 leaves an order of magnitude of margin.
//
// covar_N/C/B don't fit a fixed tolerance: they range from ~1e-7 (attitude-only) to ~1e5
// (position-uncertainty dominated, close range + Binary correction), so a single absolute bound
// would either reject rounding noise on the large entries or mean nothing on the small ones. Use
// atol + rtol*|reference| instead, same as the xmera Python test:
//   - covarRtol = 1e-4F: observed relative error on the large entries is ~1e-6, so 100x margin.
//   - covarAtol = 5e-3F: covers the near-zero off-diagonal terms left over from rotating an O(1e5)
//     matrix by an FP32 DCM (residual scales with magnitude * FP32 epsilon); observed up to ~2e-3.
//
// unitVecTimeTag gets a tight 1e-9: it's `cobTimeTag * kNano2Sec` in double on both sides, so
// there's no FP32 rounding to absorb -- just double round-off.
inline void expectNear(float actual, float reference, float atol, float rtol) {
    EXPECT_NEAR(actual, reference, atol + (rtol * std::abs(reference)));
}

inline void expectOutputsNear(const CobConverterOutput& out, const CobConverterOutput& ref, float tol) {
    constexpr float covarAtol = 5e-3F;
    constexpr float covarRtol = 1e-4F;
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(out.unitVec.rhat_BN_N(i), ref.unitVec.rhat_BN_N(i), tol);
        EXPECT_NEAR(out.unitVec.rhat_BN_C(i), ref.unitVec.rhat_BN_C(i), tol);
        EXPECT_NEAR(out.unitVec.rhat_BN_B(i), ref.unitVec.rhat_BN_B(i), tol);
        for (int j = 0; j < 3; ++j) {
            expectNear(out.unitVec.covar_N(i, j), ref.unitVec.covar_N(i, j), covarAtol, covarRtol);
            expectNear(out.unitVec.covar_C(i, j), ref.unitVec.covar_C(i, j), covarAtol, covarRtol);
            expectNear(out.unitVec.covar_B(i, j), ref.unitVec.covar_B(i, j), covarAtol, covarRtol);
        }
    }
    EXPECT_NEAR(out.unitVec.unitVecTimeTag, ref.unitVec.unitVecTimeTag, 1e-9);
    EXPECT_EQ(out.unitVec.unitVecValid, ref.unitVec.unitVecValid);

    EXPECT_NEAR(out.com.centerOfBrightness(0), ref.com.centerOfBrightness(0), tol);
    EXPECT_NEAR(out.com.centerOfBrightness(1), ref.com.centerOfBrightness(1), tol);
    EXPECT_NEAR(out.com.centerOfMass(0), ref.com.centerOfMass(0), tol);
    EXPECT_NEAR(out.com.centerOfMass(1), ref.com.centerOfMass(1), tol);
    EXPECT_NEAR(out.com.offsetFactor, ref.com.offsetFactor, tol);
    EXPECT_EQ(out.com.objectPixelRadius, ref.com.objectPixelRadius);
    EXPECT_NEAR(out.com.phaseAngle, ref.com.phaseAngle, tol);
    EXPECT_NEAR(out.com.sunDirection, ref.com.sunDirection, tol);
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
                             float fieldOfView,
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
                                         fieldOfView,
                                         resolutionX,
                                         resolutionY,
                                         bodyToCameraMrp);
    } catch (const fsw::invalid_argument&) {
        return;
    }

    const CobConverterInput input{.cobValid = cobValid,
                                  .cobPixelsFound = cobPixelsFound,
                                  .cobCenterOfBrightness = cobCenterOfBrightness,
                                  .cobTimeTag = cobTimeTag,
                                  .sigma_BN = sigma_BN,
                                  .vehSunPntBdy = vehSunPntBdy,
                                  .filterVehPosition = filterVehPosition,
                                  .filterVehPositionCovariance = filterVehPositionCovariance};

    CobConverterAlgorithm alg(*cfg);
    CobConverterOutput out;
    EXPECT_NO_THROW(out = alg.updateState(input));
    const CobConverterOutput ref = referenceCobConverterUpdate(*cfg, input);

    // See the tolerance comment above expectNear/expectOutputsNear.
    constexpr float fixedRangeTol = 1e-3F;
    expectOutputsNear(out, ref, fixedRangeTol);
}

inline void testCobConverterSetup() {
    const Eigen::Matrix3f zeroCovariance = Eigen::Matrix3f::Zero();
    const CalibrationCoefficients coefficients{};
    const Eigen::Vector3f zeroMrp = Eigen::Vector3f::Zero();

    // Builds a config from a fixed valid baseline, overriding only the fields under test below.
    const auto makeConfig = [&](PhaseAngleCorrectionMethodAlgorithm method,
                                float radius,
                                float fieldOfView,
                                int cameraId,
                                float resolutionY) {
        return CobConverterConfig::create(method,
                                          radius,
                                          8.0e3F,
                                          zeroCovariance,
                                          3.0F,
                                          100.0F,
                                          true,
                                          true,
                                          coefficients,
                                          cameraId,
                                          fieldOfView,
                                          512.0F,
                                          resolutionY,
                                          zeroMrp);
    };

    const float nan = std::numeric_limits<float>::quiet_NaN();

    // phaseAngleCorrectionMethod: only NoCorrectionAlg/BinaryAlg are valid.
    EXPECT_TRUE(
        CobConverterConfig::isValidPhaseAngleCorrectionMethod(PhaseAngleCorrectionMethodAlgorithm::NoCorrectionAlg));
    EXPECT_TRUE(CobConverterConfig::isValidPhaseAngleCorrectionMethod(PhaseAngleCorrectionMethodAlgorithm::BinaryAlg));
    EXPECT_FALSE(
        CobConverterConfig::isValidPhaseAngleCorrectionMethod(static_cast<PhaseAngleCorrectionMethodAlgorithm>(99)));

    // radius: must be > 0.
    EXPECT_TRUE(CobConverterConfig::isValidRadius(1.0F));
    EXPECT_FALSE(CobConverterConfig::isValidRadius(0.0F));
    EXPECT_FALSE(CobConverterConfig::isValidRadius(-1.0F));
    EXPECT_FALSE(CobConverterConfig::isValidRadius(nan));

    // radiusUncertainty: must be >= 0.
    EXPECT_TRUE(CobConverterConfig::isValidRadiusUncertainty(0.0F));
    EXPECT_FALSE(CobConverterConfig::isValidRadiusUncertainty(-1.0F));
    EXPECT_FALSE(CobConverterConfig::isValidRadiusUncertainty(nan));

    // attitudeCovariance: must be finite.
    EXPECT_TRUE(CobConverterConfig::isValidAttitudeCovariance(zeroCovariance));
    Eigen::Matrix3f nanCovariance = Eigen::Matrix3f::Zero();
    nanCovariance(0, 0) = nan;
    EXPECT_FALSE(CobConverterConfig::isValidAttitudeCovariance(nanCovariance));

    // numStandardDeviations: must be > 0.
    EXPECT_TRUE(CobConverterConfig::isValidNumStandardDeviations(1.0F));
    EXPECT_FALSE(CobConverterConfig::isValidNumStandardDeviations(0.0F));
    EXPECT_FALSE(CobConverterConfig::isValidNumStandardDeviations(-1.0F));

    // standardDeviation: only checked when specified.
    EXPECT_TRUE(CobConverterConfig::isValidStandardDeviation(-1.0F, /*specified=*/false));
    EXPECT_TRUE(CobConverterConfig::isValidStandardDeviation(1.0F, /*specified=*/true));
    EXPECT_FALSE(CobConverterConfig::isValidStandardDeviation(0.0F, /*specified=*/true));
    EXPECT_FALSE(CobConverterConfig::isValidStandardDeviation(-1.0F, /*specified=*/true));

    // calibrationCoefficients: must be finite.
    EXPECT_TRUE(CobConverterConfig::isValidCalibrationCoefficients(coefficients));
    CalibrationCoefficients nanCoefficients{};
    nanCoefficients.k1 = nan;
    EXPECT_FALSE(CobConverterConfig::isValidCalibrationCoefficients(nanCoefficients));

    // fieldOfView: must be in (0, pi).
    EXPECT_TRUE(CobConverterConfig::isValidFieldOfView(0.35F));
    EXPECT_FALSE(CobConverterConfig::isValidFieldOfView(0.0F));
    EXPECT_FALSE(CobConverterConfig::isValidFieldOfView(std::numbers::pi_v<float>));
    EXPECT_FALSE(CobConverterConfig::isValidFieldOfView(-0.1F));

    // resolutionX / resolutionY: must be > 0.
    EXPECT_TRUE(CobConverterConfig::isValidResolutionX(512.0F));
    EXPECT_FALSE(CobConverterConfig::isValidResolutionX(0.0F));
    EXPECT_TRUE(CobConverterConfig::isValidResolutionY(512.0F));
    EXPECT_FALSE(CobConverterConfig::isValidResolutionY(0.0F));

    // bodyToCameraMrp: must be finite.
    EXPECT_TRUE(CobConverterConfig::isValidBodyToCameraMrp(zeroMrp));
    EXPECT_FALSE(CobConverterConfig::isValidBodyToCameraMrp(Eigen::Vector3f{nan, 0.0F, 0.0F}));

    // create() throws on the first invalid field it encounters.
    EXPECT_THROW(
        makeConfig(PhaseAngleCorrectionMethodAlgorithm::NoCorrectionAlg, 0.0F /* invalid radius */, 0.35F, 0, 512.0F),
        fsw::invalid_argument);
    EXPECT_THROW(makeConfig(PhaseAngleCorrectionMethodAlgorithm::NoCorrectionAlg,
                            25.0e3F,
                            std::numbers::pi_v<float> /* invalid fieldOfView */,
                            0,
                            512.0F),
                 fsw::invalid_argument);

    // A fully valid config builds without throwing and round-trips its values through the getters.
    const CobConverterConfig cfg =
        makeConfig(PhaseAngleCorrectionMethodAlgorithm::BinaryAlg, 25.0e3F, 0.35F, 7, 256.0F);
    EXPECT_EQ(cfg.getPhaseAngleCorrectionMethod(), PhaseAngleCorrectionMethodAlgorithm::BinaryAlg);
    EXPECT_FLOAT_EQ(cfg.getRadius(), 25.0e3F);
    EXPECT_FLOAT_EQ(cfg.getRadiusUncertainty(), 8.0e3F);
    EXPECT_FLOAT_EQ(cfg.getNumStandardDeviations(), 3.0F);
    EXPECT_FLOAT_EQ(cfg.getStandardDeviation(), 100.0F);
    EXPECT_TRUE(cfg.isStandardDeviationSpecified());
    EXPECT_TRUE(cfg.isOutlierDetectionEnabled());
    EXPECT_EQ(cfg.getCameraId(), 7);
    EXPECT_FLOAT_EQ(cfg.getFieldOfView(), 0.35F);
    EXPECT_FLOAT_EQ(cfg.getResolutionX(), 512.0F);
    EXPECT_FLOAT_EQ(cfg.getResolutionY(), 256.0F);
}

#endif  // TEST_COBCONVERTER_HELPERS_H
