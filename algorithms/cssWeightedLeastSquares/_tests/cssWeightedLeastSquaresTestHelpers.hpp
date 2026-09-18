#ifndef TEST_CSS_WEIGHTED_LEAST_SQUARES_H
#define TEST_CSS_WEIGHTED_LEAST_SQUARES_H

#include "cssWeightedLeastSquaresAlgorithm.h"

#include <gtest/gtest.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

// [-] cosine at or below which a reading is treated as noise and dropped from the fit
inline constexpr float kSensorUseThresh = 0.15F;
// [s] time between two update() calls in the tests
inline constexpr float kControlPeriod = 0.5F;

// Eight-sensor constellation: two opposing four-sensor pyramids, so a sun along any body axis lights at
// least three sensors and the fit is over-determined. The Python test uses the same one.
inline std::vector<Eigen::Vector3d> referenceConstellation() {
    const double a = 0.70710678118654746;
    const double b = 0.70710678118654757;
    return {{a, -0.5, 0.5},
            {a, -0.5, -0.5},
            {a, 0.5, -0.5},
            {a, 0.5, 0.5},
            {-a, 0.0, b},
            {-a, b, 0.0},
            {-a, 0.0, -b},
            {-a, -b, 0.0}};
}

// The compacted measurement system the active sensors describe: one row of H and one entry of y per
// observation, with the weights the fit would apply. Built from the documented selection rule, not from the
// implementation.
struct ActiveSystem {
    Eigen::MatrixXd H;              // [-] one row per observation, the calibrated boresight
    Eigen::VectorXd y;              // [-] the reading behind each observation
    Eigen::VectorXd weights;        // [-] the diagonal of the weighting matrix
    std::vector<uint32_t> sensors;  // [-] the sensor behind each observation
    double condition{1.0};          // [-] conditioning of the weighted normal matrix
    bool resolvable{};              // [-] whether fp32 has significant digits left on this system
};

// The sensors that contribute to a fit: enabled by a positive bias, reporting a finite reading, and reading
// above the use threshold. Returns sensor indices in sensor order, which is also observation order.
inline std::vector<uint32_t> referenceActiveSensors(const std::vector<double>& biases,
                                                    const std::vector<double>& readings,
                                                    double sensorUseThresh) {
    std::vector<uint32_t> active;
    for (uint32_t i = 0U; i < kMaxNumCssSensors; ++i) {
        if (biases[i] > 0.0 && std::isfinite(readings[i]) && readings[i] > sensorUseThresh) {
            active.push_back(i);
        }
    }
    return active;
}

inline ActiveSystem activeSystem(const std::vector<Eigen::Vector3d>& boresights,
                                 const std::vector<double>& biases,
                                 bool useWeights,
                                 double sensorUseThresh,
                                 const std::vector<double>& readings) {
    ActiveSystem system{};
    system.sensors = referenceActiveSensors(biases, readings, sensorUseThresh);
    const auto n = static_cast<Eigen::Index>(system.sensors.size());
    if (n == 0) {
        return system;
    }

    system.H.resize(n, 3);
    system.y.resize(n);
    for (Eigen::Index k = 0; k < n; ++k) {
        const uint32_t sensor = system.sensors[static_cast<size_t>(k)];
        system.H.row(k) = biases[sensor] * boresights[sensor].transpose();
        system.y(k) = readings[sensor];
    }
    // With one or two observations the fit reproduces the measurements exactly, so the weighting drops out
    // of the optimality condition and the same expression covers every branch.
    system.weights = useWeights ? system.y : Eigen::VectorXd::Ones(n);

    const Eigen::Matrix3d normalMatrix = system.H.transpose() * system.weights.asDiagonal() * system.H;
    const Eigen::JacobiSVD<Eigen::Matrix3d> svd(normalMatrix);
    const double smallest = std::max(svd.singularValues()(2), 1e-300);
    system.condition = svd.singularValues()(0) / smallest;
    const auto epsilon = static_cast<double>(std::numeric_limits<float>::epsilon());
    system.resolvable = svd.singularValues()(0) > 0.0 && system.condition < 1.0 / (16.0 * epsilon);
    return system;
}

// Checks the reported heading against the conditions that define the fit, rather than against a second
// implementation of it. Two conditions cover all three branches without any branching of their own:
//
//   1. the weighted normal equations, H^T W (y - H d) = 0, which is what makes d the least squares fit;
//   2. d lies in the row space of H, which is what makes the underdetermined fits minimum norm.
//
// The estimator publishes the heading normalized, so the scale of the unnormalized fit is recovered first by
// projecting the measurements onto the reported direction.
inline void expectFitIsOptimal(const ActiveSystem& system,
                               const Eigen::Vector3f& sunHeading_B,
                               const Eigen::Vector<float, kMaxNumCssSensors>& postFitResiduals) {
    if (!system.resolvable || sunHeading_B.isZero()) {
        return;
    }

    const Eigen::Vector3d heading = sunHeading_B.cast<double>().normalized();
    const Eigen::Vector3d htWy = system.H.transpose() * (system.weights.asDiagonal() * system.y);
    const Eigen::Matrix3d htWh = system.H.transpose() * system.weights.asDiagonal() * system.H;

    const double denominator = heading.dot(htWh * heading);
    if (denominator <= 0.0) {
        return;  // the reported direction carries no weighted signal, so there is no scale to recover
    }
    const Eigen::Vector3d fit = (heading.dot(htWy) / denominator) * heading;

    const auto epsilon = static_cast<double>(std::numeric_limits<float>::epsilon());
    const double tolerance = std::max(1e-5, 64.0 * epsilon * system.condition);

    // 1. The weighted normal equations, measured against the scale of the data they are formed from.
    const double dataScale = std::max(1e-30, htWy.norm());
    EXPECT_LT((htWy - (htWh * fit)).norm() / dataScale, tolerance)
        << "normal equations, condition " << system.condition;

    // 2. The heading lies in the row space of H, so the underdetermined fits are the minimum norm ones.
    const Eigen::JacobiSVD<Eigen::MatrixXd> svd(system.H, Eigen::ComputeThinV);
    const double largestSingularValue = svd.singularValues()(0);
    Eigen::Vector3d inRowSpace = Eigen::Vector3d::Zero();
    for (Eigen::Index i = 0; i < svd.singularValues().size(); ++i) {
        if (svd.singularValues()(i) > largestSingularValue * 1e-6) {
            inRowSpace += svd.matrixV().col(i) * svd.matrixV().col(i).dot(heading);
        }
    }
    EXPECT_LT((heading - inRowSpace).norm(), std::max(1e-4, tolerance)) << "heading outside the row space of H";

    // The residuals follow from the fit by their own definition, so they are checked directly.
    for (size_t k = 0U; k < system.sensors.size(); ++k) {
        const uint32_t sensor = system.sensors[k];
        const Eigen::Vector3d boresight = system.H.row(static_cast<Eigen::Index>(k)).transpose().normalized();
        const double prediction = std::max(0.0, fit.dot(boresight));
        const double expected = system.y(static_cast<Eigen::Index>(k)) - prediction;
        const double residualScale = std::max(1.0, std::abs(expected));
        EXPECT_NEAR(static_cast<double>(postFitResiduals(static_cast<Eigen::Index>(k))),
                    expected,
                    4.0 * tolerance * residualScale)
            << "residual " << k << " of sensor " << sensor;
    }
}

// Inputs the tests and the fuzz domains share, kept together so a helper signature stays readable.
struct ConstellationInputs {
    std::vector<float> boresights;  // kMaxNumCssSensors * 3 components, normalized on the way in
    std::vector<float> biases;      // kMaxNumCssSensors entries, zero disables a sensor
    bool useWeights{};
    float sensorUseThresh{};
    float controlPeriod{};
};

// Everything a helper needs once the raw inputs are known to describe an acceptable configuration.
struct BuiltConfig {
    std::array<CssConfiguration, kMaxNumCssSensors> cssSensors{};
    std::vector<Eigen::Vector3d> boresights;
    std::vector<double> biases;
};

// Turns raw fuzzable inputs into a validated configuration. Returns false when the inputs cannot describe
// one the factory would accept, so the caller skips the case rather than expecting a throw.
inline bool buildConfig(const ConstellationInputs& inputs, BuiltConfig& built) {
    if (!CssWeightedLeastSquaresConfig::isValidSensorUseThresh(inputs.sensorUseThresh) ||
        !CssWeightedLeastSquaresConfig::isValidControlPeriod(inputs.controlPeriod)) {
        return false;
    }
    if (inputs.boresights.size() < static_cast<size_t>(kMaxNumCssSensors) * 3U ||
        inputs.biases.size() < static_cast<size_t>(kMaxNumCssSensors)) {
        return false;
    }

    built = BuiltConfig{};
    built.boresights.assign(static_cast<size_t>(kMaxNumCssSensors), Eigen::Vector3d::Zero());
    built.biases.assign(static_cast<size_t>(kMaxNumCssSensors), 0.0);

    for (uint32_t i = 0U; i < kMaxNumCssSensors; ++i) {
        const Eigen::Vector3f raw{
            inputs.boresights[3U * i], inputs.boresights[(3U * i) + 1U], inputs.boresights[(3U * i) + 2U]};
        const float rawNorm = raw.stableNorm();
        if (!std::isfinite(rawNorm) || rawNorm < 1e-3F) {
            return false;  // a boresight too small to normalize cannot describe a sensor
        }
        if (!std::isfinite(inputs.biases[i]) || inputs.biases[i] < 0.0F) {
            return false;
        }
        built.cssSensors.at(i).nHat_B = raw / rawNorm;
        built.cssSensors.at(i).bias = inputs.biases[i];
        // The configuration normalizes the boresights, so the reference uses the normalized ones too.
        built.boresights[i] = built.cssSensors.at(i).nHat_B.stableNormalized().cast<double>();
        built.biases[i] = static_cast<double>(inputs.biases[i]);
    }

    return CssWeightedLeastSquaresConfig::isValidCssSensors(built.cssSensors);
}

inline CssWeightedLeastSquaresConfig makeConfig(const ConstellationInputs& inputs, const BuiltConfig& built) {
    return CssWeightedLeastSquaresConfig::create(
        built.cssSensors, inputs.useWeights, inputs.sensorUseThresh, inputs.controlPeriod);
}

// Pads a reading vector out to the full sensor array.
inline Eigen::Vector<float, kMaxNumCssSensors> makeReadings(const std::vector<float>& readings) {
    Eigen::Vector<float, kMaxNumCssSensors> cosValues = Eigen::Vector<float, kMaxNumCssSensors>::Zero();
    for (size_t i = 0U; i < readings.size() && i < static_cast<size_t>(kMaxNumCssSensors); ++i) {
        cosValues(static_cast<Eigen::Index>(i)) = readings[i];
    }
    return cosValues;
}

inline std::vector<double> toDouble(const std::vector<float>& values) {
    std::vector<double> out(values.size());
    std::transform(values.begin(), values.end(), out.begin(), [](float v) { return static_cast<double>(v); });
    return out;
}

// The readings a sun heading produces on the reference constellation. A coarse sun sensor cannot report a
// negative cosine, so a sensor facing away reads zero.
inline std::vector<float> readingsFor(const Eigen::Vector3d& sunHeading_B) {
    const std::vector<Eigen::Vector3d> boresights = referenceConstellation();
    std::vector<float> readings(boresights.size(), 0.0F);
    for (size_t i = 0U; i < boresights.size(); ++i) {
        readings[i] = static_cast<float>(std::max(0.0, boresights[i].dot(sunHeading_B)));
    }
    return readings;
}

// The reference constellation as a flat boresight vector, for the helpers that take raw inputs.
inline std::vector<float> referenceBoresightVector() {
    std::vector<float> flat;
    for (const Eigen::Vector3d& boresight : referenceConstellation()) {
        flat.push_back(static_cast<float>(boresight.x()));
        flat.push_back(static_cast<float>(boresight.y()));
        flat.push_back(static_cast<float>(boresight.z()));
    }
    return flat;
}

inline std::vector<float> unitBiases() { return std::vector<float>(static_cast<size_t>(kMaxNumCssSensors), 1.0F); }

// The reference eight-sensor setup, which most fixed-input tests start from.
inline ConstellationInputs referenceInputs() {
    return ConstellationInputs{referenceBoresightVector(), unitBiases(), false, kSensorUseThresh, kControlPeriod};
}

// ---------------------------------------------------------------------------
// Regression helper — compares update() against the independent fp64 reference.
// Re-run under fuzz inputs in the fuzz file.
// ---------------------------------------------------------------------------

inline void runRegressionCase(ConstellationInputs inputs, std::vector<float> readings) {
    BuiltConfig built{};
    if (!buildConfig(inputs, built) || readings.size() < static_cast<size_t>(kMaxNumCssSensors)) {
        return;
    }

    CssWeightedLeastSquaresAlgorithm algorithm{makeConfig(inputs, built)};
    const CssWeightedLeastSquaresOutput out = algorithm.update(makeReadings(readings));

    const ActiveSystem system = activeSystem(built.boresights,
                                             built.biases,
                                             inputs.useWeights,
                                             static_cast<double>(inputs.sensorUseThresh),
                                             toDouble(readings));

    // Selecting the active sensors is pure logic, so the counts must agree exactly.
    EXPECT_EQ(out.numCssViewingSun, static_cast<uint32_t>(system.sensors.size()));

    expectFitIsOptimal(system, out.sunHeading_B, out.postFitResiduals);
}

// ---------------------------------------------------------------------------
// Property helpers — invariants that hold for every accepted configuration.
// ---------------------------------------------------------------------------

// Every published product is a finite number, whatever the readings.
inline void propertyOutputIsFinite(ConstellationInputs inputs, std::vector<float> readings) {
    BuiltConfig built{};
    if (!buildConfig(inputs, built) || readings.size() < static_cast<size_t>(kMaxNumCssSensors)) {
        return;
    }

    CssWeightedLeastSquaresAlgorithm algorithm{makeConfig(inputs, built)};
    // Two cycles, so the rate path is exercised as well as the fit.
    for (int cycle = 0; cycle < 2; ++cycle) {
        const CssWeightedLeastSquaresOutput out = algorithm.update(makeReadings(readings));
        EXPECT_TRUE(out.sunHeading_B.allFinite());
        EXPECT_TRUE(out.omega_BN_B.allFinite());
        EXPECT_TRUE(out.postFitResiduals.allFinite());
        EXPECT_LE(out.numCssViewingSun, kMaxNumCssSensors);
    }
}

// The reported heading is either a unit vector or exactly zero; there is no third state.
inline void propertyHeadingIsUnitOrZero(ConstellationInputs inputs, std::vector<float> readings) {
    BuiltConfig built{};
    if (!buildConfig(inputs, built) || readings.size() < static_cast<size_t>(kMaxNumCssSensors)) {
        return;
    }

    CssWeightedLeastSquaresAlgorithm algorithm{makeConfig(inputs, built)};
    const CssWeightedLeastSquaresOutput out = algorithm.update(makeReadings(readings));

    if (out.sunHeading_B.isZero()) {
        SUCCEED();
        return;
    }
    EXPECT_NEAR(out.sunHeading_B.norm(), 1.0F, 1e-5F);
}

// The residuals are indexed by observation, so entries at and beyond the count stay zero. The fit
// forms its products over the full-width operands and depends on that padding.
inline void propertyResidualsPaddedWithZeros(ConstellationInputs inputs, std::vector<float> readings) {
    BuiltConfig built{};
    if (!buildConfig(inputs, built) || readings.size() < static_cast<size_t>(kMaxNumCssSensors)) {
        return;
    }

    CssWeightedLeastSquaresAlgorithm algorithm{makeConfig(inputs, built)};
    const CssWeightedLeastSquaresOutput out = algorithm.update(makeReadings(readings));

    for (uint32_t k = out.numCssViewingSun; k < static_cast<uint32_t>(kMaxNumCssSensors); ++k) {
        EXPECT_EQ(out.postFitResiduals(static_cast<Eigen::Index>(k)), 0.0F) << "residual slot " << k;
    }
}

// A sensor disabled by a zero bias produces the same cycle as one whose reading the threshold rejects: it
// contributes nothing to the fit and is not counted among the sensors viewing the sun.
inline void propertyDisabledSensorIgnored(ConstellationInputs inputs,
                                          std::vector<float> readings,
                                          uint32_t disabledIndex) {
    BuiltConfig built{};
    if (!buildConfig(inputs, built) || readings.size() < static_cast<size_t>(kMaxNumCssSensors) ||
        disabledIndex >= kMaxNumCssSensors) {
        return;
    }

    // Disabling a sensor must match silencing it, so its own reading has to be one the threshold rejects.
    std::vector<float> silenced = readings;
    silenced[disabledIndex] = 0.0F;

    BuiltConfig disabledBuilt = built;
    disabledBuilt.cssSensors.at(disabledIndex).bias = 0.0F;

    CssWeightedLeastSquaresAlgorithm enabled{makeConfig(inputs, built)};
    CssWeightedLeastSquaresAlgorithm disabled{makeConfig(inputs, disabledBuilt)};

    const CssWeightedLeastSquaresOutput silencedOut = enabled.update(makeReadings(silenced));
    const CssWeightedLeastSquaresOutput disabledOut = disabled.update(makeReadings(readings));

    EXPECT_EQ(disabledOut.numCssViewingSun, silencedOut.numCssViewingSun);
    EXPECT_LT((disabledOut.sunHeading_B - silencedOut.sunHeading_B).norm(), 1e-5F);
}

// Rotating the whole constellation rotates the heading with it. The fit is a geometric construction, so it
// cannot depend on the frame the boresights happen to be written in.
inline void propertyRotationEquivariance(ConstellationInputs inputs,
                                         std::vector<float> readings,
                                         Eigen::Vector3f rotationVector) {
    BuiltConfig built{};
    if (!buildConfig(inputs, built) || readings.size() < static_cast<size_t>(kMaxNumCssSensors)) {
        return;
    }
    if (!rotationVector.allFinite() || rotationVector.stableNorm() < 1e-3F) {
        return;
    }

    const Eigen::AngleAxisf rotation{rotationVector.stableNorm(), rotationVector.stableNormalized()};
    const Eigen::Matrix3f dcm = rotation.toRotationMatrix();

    BuiltConfig rotatedBuilt = built;
    for (uint32_t i = 0U; i < kMaxNumCssSensors; ++i) {
        rotatedBuilt.cssSensors.at(i).nHat_B = (dcm * built.cssSensors.at(i).nHat_B).stableNormalized();
    }

    CssWeightedLeastSquaresAlgorithm plain{makeConfig(inputs, built)};
    CssWeightedLeastSquaresAlgorithm rotated{makeConfig(inputs, rotatedBuilt)};

    const Eigen::Vector<float, kMaxNumCssSensors> cosValues = makeReadings(readings);
    const CssWeightedLeastSquaresOutput plainOut = plain.update(cosValues);
    const CssWeightedLeastSquaresOutput rotatedOut = rotated.update(cosValues);

    EXPECT_EQ(rotatedOut.numCssViewingSun, plainOut.numCssViewingSun);
    if (plainOut.sunHeading_B.isZero() || rotatedOut.sunHeading_B.isZero()) {
        return;
    }

    // Rotating the boresights changes the rounding of every product in the solve, and an ill-conditioned
    // normal matrix amplifies that difference, so the tolerance carries its conditioning the way the
    // regression does.
    const ActiveSystem system = activeSystem(built.boresights,
                                             built.biases,
                                             inputs.useWeights,
                                             static_cast<double>(inputs.sensorUseThresh),
                                             toDouble(readings));
    const auto epsilon = static_cast<float>(std::numeric_limits<float>::epsilon());
    const auto condition = static_cast<float>(std::min(1e6, system.condition));
    const float tolerance = std::min(0.1F, std::max(1e-4F, 128.0F * epsilon * condition));
    EXPECT_LT((rotatedOut.sunHeading_B - (dcm * plainOut.sunHeading_B)).norm(), tolerance)
        << "normal matrix condition " << system.condition;
}

// The rate comes from the cross product of two headings, so it is square to the heading reported with it.
// Only the component across the sun line is observable, which is what that orthogonality states.
inline void propertyRateOrthogonalToHeading(ConstellationInputs inputs,
                                            std::vector<float> firstReadings,
                                            std::vector<float> secondReadings) {
    BuiltConfig built{};
    if (!buildConfig(inputs, built) || firstReadings.size() < static_cast<size_t>(kMaxNumCssSensors) ||
        secondReadings.size() < static_cast<size_t>(kMaxNumCssSensors)) {
        return;
    }

    CssWeightedLeastSquaresAlgorithm algorithm{makeConfig(inputs, built)};
    algorithm.update(makeReadings(firstReadings));
    const CssWeightedLeastSquaresOutput out = algorithm.update(makeReadings(secondReadings));

    if (out.omega_BN_B.isZero() || out.sunHeading_B.isZero()) {
        return;
    }

    // The axis is the cross product of two unit headings, so its direction is only resolved to about the
    // working precision divided by the sine of the angle between them. Headings that are nearly parallel,
    // or nearly opposed, therefore fix the axis loosely, and the tolerance has to say so.
    const float angle = out.omega_BN_B.stableNorm() * inputs.controlPeriod;
    const float sineOfAngle = std::max(std::sin(angle), 1e-6F);
    const auto epsilon = static_cast<float>(std::numeric_limits<float>::epsilon());
    const float tolerance = std::min(0.5F, std::max(1e-4F, 64.0F * epsilon / sineOfAngle));

    const float alignment = out.omega_BN_B.stableNormalized().dot(out.sunHeading_B);
    EXPECT_NEAR(alignment, 0.0F, tolerance) << "angle between headings " << angle;
}

#endif  // TEST_CSS_WEIGHTED_LEAST_SQUARES_H
