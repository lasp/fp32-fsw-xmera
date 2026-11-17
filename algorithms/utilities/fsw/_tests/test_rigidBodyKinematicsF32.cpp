#include "architecture/utilities/eigenSupport.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include "architecture/utilities/tests/rbk_float_wrappers.h"
#include "utilities/fsw/rigidBodyKinematics.hpp"

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <functional>
#include <limits>
#include <numbers>
#include <random>
#include <string>
#include <type_traits>
#include <vector>

/**
 * @file test_rigidBodyKinematicsF32.cpp
 * @brief Comprehensive regression tests for rigid-body kinematics utilities.
 *
 * @section scope Scope
 * Covers additive attitude operations (MRP/PRV/Euler), holonomic Euler-parameter constraints,
 * every B-matrix variant (B, Binv, Bdot), conversions between DCMs and all supported
 * parameterizations, cross-representation transforms (EP↔MRP, PRV↔Euler321, etc.),
 * time-derivative helpers, rotation-matrix builders, and tilde matrices. Randomized sampling over
 * wide angular ranges exposes singularity behavior and conditioning extremes.
 *
 * @section reference_parity Reference parity
 * Each templated C++ routine is validated against its legacy C implementation by converting
 * the C arrays back into Eigen objects, comparing outputs, and checking secondary invariants such as
 * DCM orthogonality, B·Bᵀ scaling, and Euler-parameter holonomic constraints.
 *
 * @section relative_tolerance Relative tolerance strategy
 * Rather than loosening global tolerances, tests rely on `rotationParameterTolerance(param)` to
 * scale machine epsilon by max(‖param‖, 1) and a conditioning term (including a safety factor). The
 * helper `relativeErrorNorm(actual, expected)` computes `(actual-expected).norm() /
 * (max(expected.norm(), 1) + eps)`. Most expectations bound this relative error to keep precision
 * requirements realistic near singular regions while keeping tight checks elsewhere.
 *
 * @section sections Section guide
 * - AdditiveTest/EulerParameterTest: Ensures addition/subtraction routines preserve valid states and
 *   match the C references, including holonomic constraints for Euler parameters.
 * - BMatrixTypedTest: Validates all B/Binv/Bdot implementations against structural identities
 *   (e.g., `binv * bmat ≈ I`) and reference arrays.
 * - DcmToRepresentationTest & RepresentationTransformTest: Exercise all DCM↔representation paths and
 *   cross-representation conversions while enforcing orthogonality and reference parity.
 * - RepresentationDerivativesTest: Checks differential helpers, MRP switching/shadowing, rotation
 *   matrix generation, and tilde matrices using the relative-error framework.
 */

// Test-wide Helpers
using FloatingPointTypes = ::testing::Types<float, double>;

template <typename T>
T kinematicsAccuracy() {
    if constexpr (std::is_same_v<T, float>) {
        return static_cast<T>(1e-7);
    } else {
        return static_cast<T>(1e-15);
    }
}

template <typename Derived>
typename Derived::Scalar rotationParameterTolerance(Eigen::MatrixBase<Derived> const& param) {
    using Scalar = typename Derived::Scalar;
    Scalar const eps = std::numeric_limits<Scalar>::epsilon();
    Scalar const magnitude = std::max<Scalar>(static_cast<Scalar>(1), param.norm());
    Scalar const conditioning = static_cast<Scalar>(1) + magnitude + magnitude * magnitude;
    Scalar const safetyFactor = static_cast<Scalar>(32);
    return safetyFactor * eps * conditioning;
}

template <typename DerivedA, typename DerivedB>
typename DerivedA::Scalar relativeErrorNorm(Eigen::MatrixBase<DerivedA> const& actual,
                                            Eigen::MatrixBase<DerivedB> const& expected) {
    using Scalar = typename DerivedA::Scalar;
    static_assert(std::is_same_v<Scalar, typename DerivedB::Scalar>, "Scalar mismatch in relativeErrorNorm");
    Scalar const eps = std::numeric_limits<Scalar>::epsilon();
    Scalar const scale = std::max(expected.norm(), static_cast<Scalar>(1));
    return (actual - expected).norm() / (scale + eps);
}

template <typename T>
Eigen::Map<Eigen::Matrix<T, 3, 3, Eigen::RowMajor> const> cArray33ToEigenMatrix33(T const (&array)[3][3]) {
    return Eigen::Map<Eigen::Matrix<T, 3, 3, Eigen::RowMajor> const>(&array[0][0]);
}

template <typename T>
Eigen::Map<Eigen::Matrix<T, 3, 4, Eigen::RowMajor> const> cArray34ToEigenMatrix34(T const (&array)[3][4]) {
    return Eigen::Map<Eigen::Matrix<T, 3, 4, Eigen::RowMajor> const>(&array[0][0]);
}

template <typename T>
Eigen::Map<Eigen::Matrix<T, 4, 3, Eigen::RowMajor> const> cArray43ToEigenMatrix43(T const (&array)[4][3]) {
    return Eigen::Map<Eigen::Matrix<T, 4, 3, Eigen::RowMajor> const>(&array[0][0]);
}

template <typename Derived, typename T>
void eigenMatrix3ToCArray(Eigen::MatrixBase<Derived> const& mat, T out[3][3]) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            out[i][j] = mat(i, j);
        }
    }
}

// Addition and subtraction of kinematic representation tests
template <typename T>
struct AdditiveTestCase {
    std::string name;
    std::function<void(T*, T*, T*)> cFunc;
    std::function<Eigen::Matrix<T, 3, 1>(Eigen::Matrix<T, 3, 1>, Eigen::Matrix<T, 3, 1>)> cppFunc;
    std::function<Eigen::Matrix<T, 3, 3>(Eigen::Matrix<T, 3, 1>)> dcmFunc;
};

template <typename T>
std::ostream& operator<<(std::ostream& os, AdditiveTestCase<T> const& param) {
    os << param.name;
#if defined(__cpp_rtti)
    if constexpr (std::is_same_v<T, float>) {
        os << "_float";
    } else if constexpr (std::is_same_v<T, double>) {
        os << "_double";
    }
#endif
    return os;
}

template <typename T>
class AdditiveTest : public ::testing::TestWithParam<AdditiveTestCase<T>> {
   public:
    using Vec3 = Eigen::Matrix<T, 3, 1>;
    using Mat3 = Eigen::Matrix<T, 3, 3>;

    std::default_random_engine generator{std::random_device{}()};
    std::uniform_real_distribution<T> dist{-std::numbers::pi, std::numbers::pi};
    T const accuracy = kinematicsAccuracy<T>();

    Vec3 randVec3() { return Vec3(dist(generator), dist(generator), dist(generator)); }
};

template <typename T>
std::string GetTypeName();

template <>
std::string GetTypeName<float>() {
    return "float";
}

template <>
std::string GetTypeName<double>() {
    return "double";
}

template <typename T>
std::string MakeNameWithType(AdditiveTestCase<T> const& testCase) {
    std::string name;
    for (char c : testCase.name) {
        if (std::isalnum(c) || c == '_') {
            name += c;
        } else {
            name += '_';
        }
    }
    return name + "_" + GetTypeName<T>();
}

std::string AdditiveTestNameFloat(::testing::TestParamInfo<AdditiveTestCase<float>> const& info) {
    return MakeNameWithType<float>(info.param);
}

std::string AdditiveTestNameDouble(::testing::TestParamInfo<AdditiveTestCase<double>> const& info) {
    return MakeNameWithType<double>(info.param);
}

template <typename T>
void runAdditiveTest(AdditiveTestCase<T> const& param,
                     Eigen::Matrix<T, 3, 1> const& a,
                     Eigen::Matrix<T, 3, 1> const& b,
                     T accuracy) {
    Eigen::Matrix<T, 3, 1> expected;
    T expectedArray[3] = {};
    param.cFunc(const_cast<T*>(a.data()), const_cast<T*>(b.data()), expectedArray);
    expected = Eigen::Matrix<T, 3, 1>(expectedArray);

    Eigen::Matrix<T, 3, 1> result = param.cppFunc(a, b);
    Eigen::Matrix<T, 3, 3> dcm = param.dcmFunc(result);

    auto const identity = Eigen::Matrix<T, 3, 3>::Identity();
    T const orthoTolerance = std::max(rotationParameterTolerance(result), accuracy);
    EXPECT_LT(relativeErrorNorm(dcm * dcm.transpose(), identity), orthoTolerance);

    T const valueTolerance = std::max(rotationParameterTolerance(expected), accuracy);
    EXPECT_LT(relativeErrorNorm(result, expected), valueTolerance);
}

class AdditiveFloatTest : public AdditiveTest<float> {};

TEST_P(AdditiveFloatTest, AdditiveProperties) {
    auto const& param = GetParam();
    Vec3 a = this->randVec3();
    Vec3 b = this->randVec3();
    runAdditiveTest<float>(param, a, b, accuracy);
}

class AdditiveDoubleTest : public AdditiveTest<double> {};

TEST_P(AdditiveDoubleTest, AdditiveProperties) {
    auto const& param = GetParam();
    Vec3 a = this->randVec3();
    Vec3 b = this->randVec3();
    runAdditiveTest<double>(param, a, b, accuracy);
}

INSTANTIATE_TEST_SUITE_P(
    FloatAddTests,
    AdditiveFloatTest,
    ::testing::Values(
        AdditiveTestCase<float>{"addMRP",
                                addMRP_float,
                                [](Eigen::Vector3f const& a, Eigen::Vector3f const& b) { return addMrp<float>(a, b); },
                                [](Eigen::Vector3f const& a) { return mrpToDcm<float>(a); }},
        AdditiveTestCase<float>{"addPrvs",
                                addPRV_float,
                                [](Eigen::Vector3f const& a, Eigen::Vector3f const& b) { return addPrv<float>(a, b); },
                                [](Eigen::Vector3f const& a) { return prvToDcm<float>(a); }},
        AdditiveTestCase<float>{
            "addEulerAngles",
            addEuler321_float,
            [](Eigen::Vector3f const& a, Eigen::Vector3f const& b) { return addEulerAngles321<float>(a, b); },
            [](Eigen::Vector3f const& a) { return eulerAngles321ToDcm<float>(a); }},
        AdditiveTestCase<float>{"subMRPs",
                                subMRP_float,
                                [](Eigen::Vector3f const& a, Eigen::Vector3f const& b) { return subMrp<float>(a, b); },
                                [](Eigen::Vector3f const& a) { return mrpToDcm<float>(a); }},
        AdditiveTestCase<float>{"subPrvs",
                                subPRV_float,
                                [](Eigen::Vector3f const& a, Eigen::Vector3f const& b) { return subPrv<float>(a, b); },
                                [](Eigen::Vector3f const& a) { return prvToDcm<float>(a); }},
        AdditiveTestCase<float>{
            "subEulerAngles",
            subEuler321_float,
            [](Eigen::Vector3f const& a, Eigen::Vector3f const& b) { return subEulerAngles321<float>(a, b); },
            [](Eigen::Vector3f const& a) { return eulerAngles321ToDcm<float>(a); }}),
    AdditiveTestNameFloat);

INSTANTIATE_TEST_SUITE_P(
    DoubleAddTests,
    AdditiveDoubleTest,
    ::testing::Values(AdditiveTestCase<double>{
                          "addMRP",
                          addMRP,
                          [](Eigen::Vector3d const& a, Eigen::Vector3d const& b) { return addMrp<double>(a, b); },
                          [](Eigen::Vector3d const& a) { return mrpToDcm<double>(a); }},
                      AdditiveTestCase<double>{
                          "addPrvs",
                          addPRV,
                          [](Eigen::Vector3d const& a, Eigen::Vector3d const& b) { return addPrv<double>(a, b); },
                          [](Eigen::Vector3d const& a) { return prvToDcm<double>(a); }},
                      AdditiveTestCase<double>{"addEulerAngles",
                                               addEuler321,
                                               [](Eigen::Vector3d const& a, Eigen::Vector3d const& b) {
                                                   return addEulerAngles321<double>(a, b);
                                               },
                                               [](Eigen::Vector3d const& a) { return eulerAngles321ToDcm<double>(a); }},
                      AdditiveTestCase<double>{
                          "subMRPs",
                          subMRP,
                          [](Eigen::Vector3d const& a, Eigen::Vector3d const& b) { return subMrp<double>(a, b); },
                          [](Eigen::Vector3d const& a) { return mrpToDcm<double>(a); }},
                      AdditiveTestCase<double>{
                          "subPrvs",
                          subPRV,
                          [](Eigen::Vector3d const& a, Eigen::Vector3d const& b) { return subPrv<double>(a, b); },
                          [](Eigen::Vector3d const& a) { return prvToDcm<double>(a); }},
                      AdditiveTestCase<double>{
                          "subEulerAngles",
                          subEuler321,
                          [](Eigen::Vector3d const& a, Eigen::Vector3d const& b) {
                              return subEulerAngles321<double>(a, b);
                          },
                          [](Eigen::Vector3d const& a) { return eulerAngles321ToDcm<double>(a); }}),
    AdditiveTestNameDouble);

// Euler Parameters add/subtract tests
template <typename T>
struct EulerTestCase {
    std::string name;
    void (*cFunc)(T*, T*, T*);
    std::function<Eigen::Matrix<T, 4, 1>(Eigen::Matrix<T, 4, 1>, Eigen::Matrix<T, 4, 1>)> cppFunc;
};

template <typename T>
std::ostream& operator<<(std::ostream& os, EulerTestCase<T> const& param) {
    os << param.name;
    return os;
}

template <typename T>
class EulerParameterTest : public ::testing::TestWithParam<EulerTestCase<T>> {
   public:
    using Vec3 = Eigen::Matrix<T, 3, 1>;
    using Vec4 = Eigen::Matrix<T, 4, 1>;

    T const accuracy = kinematicsAccuracy<T>();

    std::default_random_engine generator{std::random_device{}()};
    std::uniform_real_distribution<T> angleDistribution{-std::numbers::pi, std::numbers::pi};

    Vec4 randomEulerParameter() {
        T phi = angleDistribution(generator);
        Vec3 axis(angleDistribution(generator), angleDistribution(generator), angleDistribution(generator));
        axis.normalize();
        return Vec4(
            std::cos(phi / 2), std::sin(phi / 2) * axis(0), std::sin(phi / 2) * axis(1), std::sin(phi / 2) * axis(2));
    }

    T normConstraintViolation(Vec4 const& ep) { return std::abs(ep.squaredNorm() - static_cast<T>(1)); }

    void runTest(EulerTestCase<T> const& param) {
        Vec4 ep1 = randomEulerParameter();
        Vec4 ep2 = randomEulerParameter();

        EXPECT_LT(normConstraintViolation(ep1), std::max(rotationParameterTolerance(ep1), this->accuracy));
        EXPECT_LT(normConstraintViolation(ep2), std::max(rotationParameterTolerance(ep2), this->accuracy));

        T expectedArray[4] = {};
        param.cFunc(ep1.data(), ep2.data(), expectedArray);
        Vec4 expected = Eigen::Map<Vec4>(expectedArray);

        Vec4 result = param.cppFunc(ep1, ep2);

        EXPECT_LT(normConstraintViolation(result), std::max(rotationParameterTolerance(result), this->accuracy))
            << "Holonomic constraint violated for test: " << param.name;

        T const valueTolerance = std::max(rotationParameterTolerance(expected), this->accuracy);
        EXPECT_LT(relativeErrorNorm(result, expected), valueTolerance) << "C/C++ mismatch for test: " << param.name;
    }
};

using FloatEulerTest = EulerParameterTest<float>;
using DoubleEulerTest = EulerParameterTest<double>;

std::vector<EulerTestCase<float>> getFloatEulerTests() {
    return {
        {"addEulerParameters", addEP_float, addEp<float>},
        {"subEulerParameters", subEP_float, subEp<float>},
    };
}

std::vector<EulerTestCase<double>> getDoubleEulerTests() {
    return {
        {"addEulerParameters", addEP, addEp<double>},
        {"subEulerParameters", subEP, subEp<double>},
    };
}

TEST_P(FloatEulerTest, HolonomicConstraintAndCReferenceMatch) { this->runTest(GetParam()); }

TEST_P(DoubleEulerTest, HolonomicConstraintAndCReferenceMatch) { this->runTest(GetParam()); }

std::string EulerTestNameFloat(::testing::TestParamInfo<EulerTestCase<float>> const& info) {
    std::string name = info.param.name;
    for (char& c : name) {
        if (!std::isalnum(c)) {
            c = '_';
        }
    }
    return name + "_float";
}

std::string EulerTestNameDouble(::testing::TestParamInfo<EulerTestCase<double>> const& info) {
    std::string name = info.param.name;
    for (char& c : name) {
        if (!std::isalnum(c)) {
            c = '_';
        }
    }
    return name + "_double";
}

INSTANTIATE_TEST_SUITE_P(FloatTests, FloatEulerTest, ::testing::ValuesIn(getFloatEulerTests()), EulerTestNameFloat);

INSTANTIATE_TEST_SUITE_P(DoubleTests, DoubleEulerTest, ::testing::ValuesIn(getDoubleEulerTests()), EulerTestNameDouble);

// Bmatrix tests
template <typename T>
class BMatrixTypedTest : public ::testing::Test {
   public:
    using Vec3 = Eigen::Matrix<T, 3, 1>;
    using Vec4 = Eigen::Matrix<T, 4, 1>;
    using Mat3 = Eigen::Matrix<T, 3, 3>;
    using Mat43 = Eigen::Matrix<T, 4, 3>;

    std::default_random_engine generator{std::random_device{}()};
    std::uniform_real_distribution<T> dist{-std::numbers::pi, std::numbers::pi};
    T const accuracy = kinematicsAccuracy<T>();

    Vec3 randVec3() { return Vec3(dist(generator), dist(generator), dist(generator)); }

    Vec4 randEp() {
        Vec3 axis = randVec3().normalized();
        T phi = dist(generator);
        return Vec4(
            std::cos(phi / 2), std::sin(phi / 2) * axis(0), std::sin(phi / 2) * axis(1), std::sin(phi / 2) * axis(2));
    }
};

TYPED_TEST_SUITE(BMatrixTypedTest, FloatingPointTypes);

TYPED_TEST(BMatrixTypedTest, BinvEp) {
    auto ep = this->randEp();
    TypeParam expectedArray[3][4] = {};
    if constexpr (std::is_same_v<TypeParam, float>) {
        BinvEP_float(ep.data(), expectedArray);
    } else {
        BinvEP(ep.data(), expectedArray);
    }
    auto expected = cArray34ToEigenMatrix34(expectedArray);
    auto const binv = binvEp(ep);
    TypeParam const tolerance = rotationParameterTolerance(ep);
    TypeParam const relativeError =
        (binv - expected).norm() / (expected.norm() + std::numeric_limits<TypeParam>::epsilon());
    EXPECT_LT(relativeError, tolerance);
}

TYPED_TEST(BMatrixTypedTest, BinvMrp) {
    auto mrp = this->randVec3();
    TypeParam expectedArray[3][3] = {};
    if constexpr (std::is_same_v<TypeParam, float>) {
        BinvMRP_float(mrp.data(), expectedArray);
    } else {
        BinvMRP(mrp.data(), expectedArray);
    }
    auto expected = cArray33ToEigenMatrix33(expectedArray);

    TypeParam const tolerance = rotationParameterTolerance(mrp);
    TypeParam const eps = std::numeric_limits<TypeParam>::epsilon();
    Eigen::Matrix<TypeParam, 3, 3> const scaledBinv = (1 + mrp.dot(mrp)) * (1 + mrp.dot(mrp)) * binvMrp(mrp);
    Eigen::Matrix<TypeParam, 3, 3> const transposeBmat = bmatMrp(mrp).transpose();
    TypeParam const scaledRelativeError = (scaledBinv - transposeBmat).norm() / (transposeBmat.norm() + eps);
    EXPECT_LT(scaledRelativeError, tolerance);

    auto m = binvMrp(mrp);
    TypeParam const relativeBinvError = (m - expected).norm() / (expected.norm() + eps);
    EXPECT_LT(relativeBinvError, tolerance);
}

TYPED_TEST(BMatrixTypedTest, BinvPrv) {
    auto prv = this->randVec3();
    TypeParam const relativeTolerance = rotationParameterTolerance(prv);
    TypeParam expectedArray[3][3] = {};
    if constexpr (std::is_same_v<TypeParam, float>) {
        BinvPRV_float(prv.data(), expectedArray);
    } else {
        BinvPRV(prv.data(), expectedArray);
    }
    auto expected = cArray33ToEigenMatrix33(expectedArray);

    Eigen::Matrix<TypeParam, 3, 3> const binv = binvPrv(prv);
    Eigen::Matrix<TypeParam, 3, 3> const bmat = bmatPrv(prv);
    Eigen::Matrix<TypeParam, 3, 3> const product = binv * bmat;

    TypeParam const traceError = std::abs(product.trace() - static_cast<TypeParam>(3.0));
    EXPECT_LT(traceError / static_cast<TypeParam>(3.0), relativeTolerance);

    auto const identity = Eigen::Matrix<TypeParam, 3, 3>::Identity();
    TypeParam const relativeIdentityError = (product - identity).norm() / identity.norm();
    EXPECT_LT(relativeIdentityError, relativeTolerance);

    TypeParam const relativeBinvError =
        (binv - expected).norm() / (expected.norm() + std::numeric_limits<TypeParam>::epsilon());
    EXPECT_LT(relativeBinvError, relativeTolerance);
}

TYPED_TEST(BMatrixTypedTest, BinvEuler321) {
    auto euler = this->randVec3();
    TypeParam expectedArray[3][3] = {};
    if constexpr (std::is_same_v<TypeParam, float>) {
        BinvEuler321_float(euler.data(), expectedArray);
    } else {
        BinvEuler321(euler.data(), expectedArray);
    }
    auto expected = cArray33ToEigenMatrix33(expectedArray);
    auto const binv = binvEulerAngles321(euler);
    TypeParam const tolerance = rotationParameterTolerance(euler);
    TypeParam const relativeError =
        (binv - expected).norm() / (expected.norm() + std::numeric_limits<TypeParam>::epsilon());
    EXPECT_LT(relativeError, tolerance);
}

TYPED_TEST(BMatrixTypedTest, BmatEp) {
    auto ep = this->randEp();
    TypeParam expectedArray[4][3] = {};
    if constexpr (std::is_same_v<TypeParam, float>) {
        BmatEP_float(ep.data(), expectedArray);
    } else {
        BmatEP(ep.data(), expectedArray);
    }
    auto expected = cArray43ToEigenMatrix43(expectedArray);
    auto const bmat = bmatEp(ep);
    TypeParam const tolerance = rotationParameterTolerance(ep);
    TypeParam const eps = std::numeric_limits<TypeParam>::epsilon();
    TypeParam const orthogonalityError = (bmat.transpose() * ep).norm() / (bmat.norm() + eps);
    EXPECT_LT(orthogonalityError, tolerance);
    TypeParam const relativeError = (bmat - expected).norm() / (expected.norm() + eps);
    EXPECT_LT(relativeError, tolerance);
}

// ---------------------------------------------------------------------------
// bmatMrp: computes [B(sigma)] = (1 - |sigma|^2) I + 2 [sigma~] + 2 sigma sigma^T.
// ---------------------------------------------------------------------------
TYPED_TEST(BMatrixTypedTest, bmatMrpMatchesCReference) {
    auto mrp = this->randVec3();
    TypeParam expectedArray[3][3] = {};
    if constexpr (std::is_same_v<TypeParam, float>) {
        BmatMRP_float(mrp.data(), expectedArray);
    } else {
        BmatMRP(mrp.data(), expectedArray);
    }
    auto expected = cArray33ToEigenMatrix33(expectedArray);

    auto const bmat = bmatMrp(mrp);
    TypeParam const tolerance = rotationParameterTolerance(mrp);
    TypeParam const eps = std::numeric_limits<TypeParam>::epsilon();
    Eigen::Matrix<TypeParam, 3, 3> const product = bmat.transpose() * bmat;
    Eigen::Matrix<TypeParam, 3, 3> const theoretical =
        (1 + mrp.dot(mrp)) * (1 + mrp.dot(mrp)) * Eigen::Matrix<TypeParam, 3, 3>::Identity();
    TypeParam const productError = (product - theoretical).norm() / (theoretical.norm() + eps);
    EXPECT_LT(productError, tolerance);

    TypeParam const relativeError = (bmat - expected).norm() / (expected.norm() + eps);
    EXPECT_LT(relativeError, tolerance);
}

// At the zero MRP set, [B] is the identity.
TYPED_TEST(BMatrixTypedTest, bmatMrpAtZeroIsIdentity) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;
    using Mat3 = Eigen::Matrix3<T>;

    Vec3 const zeroMrp = Vec3::Zero();
    EXPECT_EQ(bmatMrp(zeroMrp), Mat3::Identity());
}

// Verify the B-matrix property B(sigma) * sigma = (1 + |sigma|^2) * sigma.
TYPED_TEST(BMatrixTypedTest, bmatMrpTimesMrp) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;

    Vec3 const mrp(T(0.1), T(-0.2), T(0.3));
    Vec3 const expected = (T(1) + mrp.squaredNorm()) * mrp;

    EXPECT_TRUE((bmatMrp(mrp) * mrp).isApprox(expected, T(10) * std::numeric_limits<T>::epsilon()));
}

// Verify the cross-product sign using B(0.5 * X) * Y = (0, 0.75, 1),
// consistent with the right-hand rule X cross Y = Z.
TYPED_TEST(BMatrixTypedTest, bmatMrpCrossTermSign) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;

    Vec3 const mrp(T(0.5), T(0), T(0));
    Vec3 const unitY = Vec3::UnitY();
    Vec3 const expected(T(0), T(0.75), T(1));

    EXPECT_TRUE((bmatMrp(mrp) * unitY).isApprox(expected, T(10) * std::numeric_limits<T>::epsilon()));
}

// [B] [B]^T = (1 + |sigma|^2)^2 I, so [B] is a scaled rotation matrix.
TYPED_TEST(BMatrixTypedTest, bmatMrpScaledOrthogonal) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;
    using Mat3 = Eigen::Matrix3<T>;

    Vec3 const mrp(T(0.1), T(-0.2), T(0.3));
    Mat3 const bmat = bmatMrp(mrp);
    T const scale = (T(1) + mrp.squaredNorm()) * (T(1) + mrp.squaredNorm());

    EXPECT_TRUE(
        (bmat * bmat.transpose()).isApprox(scale * Mat3::Identity(), T(10) * std::numeric_limits<T>::epsilon()));
}

TYPED_TEST(BMatrixTypedTest, BdotMrp) {
    auto mrp = this->randVec3();
    auto dmrp = this->randVec3();
    TypeParam expectedArray[3][3] = {};
    if constexpr (std::is_same_v<TypeParam, float>) {
        BdotmatMRP_float(mrp.data(), dmrp.data(), expectedArray);
    } else {
        BdotmatMRP(mrp.data(), dmrp.data(), expectedArray);
    }
    auto expected = cArray33ToEigenMatrix33(expectedArray);

    TypeParam const tolerance = std::max(rotationParameterTolerance(mrp), rotationParameterTolerance(dmrp));
    TypeParam const eps = std::numeric_limits<TypeParam>::epsilon();
    TypeParam const relativeError = (bmatDotMrp(mrp, dmrp) - expected).norm() / (expected.norm() + eps);
    EXPECT_LT(relativeError, tolerance);
}

TYPED_TEST(BMatrixTypedTest, BmatPrv) {
    auto prv = this->randVec3();
    TypeParam expectedArray[3][3] = {};
    if constexpr (std::is_same_v<TypeParam, float>) {
        BmatPRV_float(prv.data(), expectedArray);
    } else {
        BmatPRV(prv.data(), expectedArray);
    }
    auto expected = cArray33ToEigenMatrix33(expectedArray);
    auto const bmat = bmatPrv(prv);
    TypeParam const tolerance = rotationParameterTolerance(prv);
    TypeParam const eps = std::numeric_limits<TypeParam>::epsilon();
    TypeParam const relativeError = (bmat - expected).norm() / (expected.norm() + eps);
    EXPECT_LT(relativeError, tolerance);
}

TYPED_TEST(BMatrixTypedTest, BmatEuler321) {
    auto euler = this->randVec3();
    TypeParam expectedArray[3][3] = {};
    if constexpr (std::is_same_v<TypeParam, float>) {
        BmatEuler321_float(euler.data(), expectedArray);
    } else {
        BmatEuler321(euler.data(), expectedArray);
    }
    auto expected = cArray33ToEigenMatrix33(expectedArray);
    auto const bmat = bmatEulerAngles321(euler);
    TypeParam const tolerance = rotationParameterTolerance(euler);
    TypeParam const eps = std::numeric_limits<TypeParam>::epsilon();
    TypeParam const relativeError = (bmat - expected).norm() / (expected.norm() + eps);
    EXPECT_LT(relativeError, tolerance);
}

// DCM to representations test
template <typename T>
class DcmToRepresentationTest : public ::testing::Test {
   public:
    using Mat3 = Eigen::Matrix<T, 3, 3>;
    using Vec3 = Eigen::Matrix<T, 3, 1>;

    std::default_random_engine generator{std::random_device{}()};
    std::uniform_real_distribution<T> dist{-std::numbers::pi, std::numbers::pi};
    T const accuracy = kinematicsAccuracy<T>();
};

TYPED_TEST_SUITE(DcmToRepresentationTest, FloatingPointTypes);

// ---------------------------------------------------------------------------
// dcmToMrp: converts a DCM to the principal MRP set with norm at most 1.
// ---------------------------------------------------------------------------
TYPED_TEST(DcmToRepresentationTest, dcmToMrpMatchesCReference) {
    using T = TypeParam;
    typename TestFixture::Mat3 dcm1 = rotationMatrix(this->dist(this->generator), 1);
    typename TestFixture::Mat3 dcm2 = rotationMatrix(this->dist(this->generator), 2);
    typename TestFixture::Mat3 dcm3 = rotationMatrix(this->dist(this->generator), 3);
    typename TestFixture::Mat3 dcm = dcm3 * dcm2 * dcm1;

    T dcmArray[3][3];
    eigenMatrix3ToCArray(dcm, dcmArray);
    T expectedArray[3] = {};
    if constexpr (std::is_same_v<T, float>) {
        C2MRP_float(dcmArray, expectedArray);
    } else {
        C2MRP(dcmArray, expectedArray);
    }
    typename TestFixture::Vec3 expected = Eigen::Map<typename TestFixture::Vec3>(expectedArray);

    auto const identity = TestFixture::Mat3::Identity();
    T const orthoTolerance = std::max(rotationParameterTolerance(dcm), this->accuracy);
    EXPECT_LT(relativeErrorNorm(dcm * dcm.transpose(), identity), orthoTolerance);

    T const valueTolerance = std::max(rotationParameterTolerance(expected), this->accuracy);
    EXPECT_LT(relativeErrorNorm(dcmToMrp(dcm), expected), valueTolerance);
}

// The identity DCM gives the zero MRP.
TYPED_TEST(DcmToRepresentationTest, dcmToMrpIdentityIsZero) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;
    using Mat3 = Eigen::Matrix3<T>;

    Mat3 const identity = Mat3::Identity();
    EXPECT_EQ(dcmToMrp(identity), Vec3::Zero());
}

// Verify DCM-to-MRP conversion for a 90-degree rotation about X.
TYPED_TEST(DcmToRepresentationTest, dcmToMrp90DegX) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;
    using Mat3 = Eigen::Matrix3<T>;

    T const tol = T(10) * std::numeric_limits<T>::epsilon();

    // DCM representing a 90-degree rotation about X.
    Mat3 dcm;
    dcm << T(1), T(0), T(0), T(0), T(0), T(1), T(0), T(-1), T(0);

    // Calculate the expected MRP using sigma = axis * tan(phi / 4).
    Vec3 const axis(T(1), T(0), T(0));
    T const phi = T(90) * std::numbers::pi_v<T> / T(180);
    Vec3 const expected = axis * std::tan(phi / T(4));

    // Verify the converted MRP against the analytical result.
    Vec3 const result = dcmToMrp(dcm);
    EXPECT_TRUE(result.isApprox(expected, tol));
}

// A 270-degree rotation about X is equivalent to a -90-degree rotation
// about X. dcmToMrp must return the principal (shortest-rotation) MRP set,
// whose norm is at most 1.
TYPED_TEST(DcmToRepresentationTest, dcmToMrp270DegX) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;
    using Mat3 = Eigen::Matrix3<T>;

    Mat3 dcm270AboutX;
    dcm270AboutX << T(1), T(0), T(0), T(0), T(0), T(-1), T(0), T(1), T(0);

    // Calculate the expected MRP using sigma = axis * tan(phi / 4).
    // The equivalent shortest rotation is 90 degrees about the negative X-axis.
    Vec3 const principalAxis(T(-1), T(0), T(0));
    T const principalAngle = T(90) * std::numbers::pi_v<T> / T(180);
    Vec3 const expectedMrp = principalAxis * std::tan(principalAngle / T(4));

    // Verify that the 270-degree DCM produces the MRP of the equivalent -90-degree rotation.
    Vec3 const result = dcmToMrp(dcm270AboutX);
    EXPECT_TRUE(result.isApprox(expectedMrp, T(10) * std::numeric_limits<T>::epsilon()));
}

// Verify 180-degree rotations about each axis.
TYPED_TEST(DcmToRepresentationTest, dcmToMrp180Degrees) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;
    using Mat3 = Eigen::Matrix<T, 3, 3>;

    Mat3 const dcm180AboutX = Vec3(T(1), T(-1), T(-1)).asDiagonal();
    Mat3 const dcm180AboutY = Vec3(T(-1), T(1), T(-1)).asDiagonal();
    Mat3 const dcm180AboutZ = Vec3(T(-1), T(-1), T(1)).asDiagonal();

    // At 180 degrees, sigma = axis * tan(phi / 4) = +/-axis.
    EXPECT_EQ(dcmToMrp(dcm180AboutX).cwiseAbs(), Vec3::UnitX());
    EXPECT_EQ(dcmToMrp(dcm180AboutY).cwiseAbs(), Vec3::UnitY());
    EXPECT_EQ(dcmToMrp(dcm180AboutZ).cwiseAbs(), Vec3::UnitZ());
}

// Verify that dcmToMrp preserves a very small rotation about the Z-axis.
// At 1e-8 radians, cos(angle) rounds to 1 in float and double, so the
// rotation is mainly represented by the off-diagonal DCM elements.
// The expected MRP is nonzero, approximately [0, 0, 2.5e-9].
TYPED_TEST(DcmToRepresentationTest, dcmToMrpSmallRotation) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;
    using Mat3 = Eigen::Matrix<T, 3, 3>;

    T const angle = T(1e-8);
    Vec3 const axis(T(0), T(0), T(1));

    // Construct the DCM for a small rotation about Z.
    Mat3 dcm;
    dcm << std::cos(angle), std::sin(angle), T(0), -std::sin(angle), std::cos(angle), T(0), T(0), T(0), T(1);

    // Calculate the expected MRP using sigma = axis * tan(angle / 4).
    Vec3 const expected = axis * std::tan(angle / T(4));

    // Verify that the small rotation is preserved instead of returning zero.
    Vec3 const result = dcmToMrp(dcm);
    EXPECT_TRUE(result.isApprox(expected, T(10) * std::numeric_limits<T>::epsilon()));
}

// ---------------------------------------------------------------------------
// mrpToDcm: converts an MRP set to its direction cosine matrix.
// ---------------------------------------------------------------------------
TYPED_TEST(DcmToRepresentationTest, mrpToDcmMatchesCReference) {
    using T = TypeParam;
    using Vec3 = Eigen::Matrix<T, 3, 1>;
    using Mat3 = Eigen::Matrix<T, 3, 3>;

    Vec3 mrp(this->dist(this->generator), this->dist(this->generator), this->dist(this->generator));
    T expectedArray[3][3] = {};
    if constexpr (std::is_same_v<T, float>) {
        MRP2C_float(mrp.data(), expectedArray);
    } else {
        MRP2C(mrp.data(), expectedArray);
    }

    Mat3 expected = cArray33ToEigenMatrix33(expectedArray);
    Mat3 result = mrpToDcm<T>(mrp);

    auto const identity = Mat3::Identity();
    T const orthoTolerance = std::max(rotationParameterTolerance(result), this->accuracy);
    EXPECT_LT(relativeErrorNorm(result * result.transpose(), identity), orthoTolerance);

    T const valueTolerance = std::max(rotationParameterTolerance(expected), this->accuracy);
    EXPECT_LT(relativeErrorNorm(result, expected), valueTolerance);
}

// The zero MRP gives the identity DCM.
TYPED_TEST(DcmToRepresentationTest, mrpToDcmZeroIsIdentity) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;
    using Mat3 = Eigen::Matrix3<T>;

    Vec3 const zeroMrp = Vec3::Zero();
    Mat3 const identity = Mat3::Identity();
    EXPECT_EQ(mrpToDcm(zeroMrp), identity);
}

// Verify MRP-to-DCM conversion for a 90-degree rotation about X.
TYPED_TEST(DcmToRepresentationTest, mrpToDcm90DegX) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;
    using Mat3 = Eigen::Matrix3<T>;

    // Calculate the MRP using sigma = axis * tan(phi / 4).
    Vec3 const axis(T(1), T(0), T(0));
    T const phi = T(90) * std::numbers::pi_v<T> / T(180);
    Vec3 const mrp = axis * std::tan(phi / T(4));

    // DCM representing a 90-degree rotation about X.
    Mat3 expected;
    expected << T(1), T(0), T(0), T(0), T(0), T(1), T(0), T(-1), T(0);

    EXPECT_TRUE(mrpToDcm(mrp).isApprox(expected, T(10) * std::numeric_limits<T>::epsilon()));
}

// A non-principal MRP (norm above 1) for a 270-degree rotation about X gives the same DCM as the -90-degree rotation.
TYPED_TEST(DcmToRepresentationTest, mrpToDcm270DegX) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;
    using Mat3 = Eigen::Matrix3<T>;

    // Calculate the MRP using sigma = axis * tan(phi / 4); its norm tan(67.5 deg) is about 2.414.
    Vec3 const axis(T(1), T(0), T(0));
    T const phi = T(270) * std::numbers::pi_v<T> / T(180);
    Vec3 const mrp = axis * std::tan(phi / T(4));

    // DCM representing a 270-degree (equivalently -90-degree) rotation about X.
    Mat3 expected;
    expected << T(1), T(0), T(0), T(0), T(0), T(-1), T(0), T(1), T(0);

    EXPECT_TRUE(mrpToDcm(mrp).isApprox(expected, T(10) * std::numeric_limits<T>::epsilon()));
}

// Verify 180-degree rotations about each principal axis.
TYPED_TEST(DcmToRepresentationTest, mrpToDcm180Degrees) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;
    using Mat3 = Eigen::Matrix3<T>;

    Vec3 const mrp180AboutX = Vec3::UnitX();
    Vec3 const mrp180AboutY = Vec3::UnitY();
    Vec3 const mrp180AboutZ = Vec3::UnitZ();

    Mat3 const dcm180AboutX = Vec3(T(1), T(-1), T(-1)).asDiagonal();
    Mat3 const dcm180AboutY = Vec3(T(-1), T(1), T(-1)).asDiagonal();
    Mat3 const dcm180AboutZ = Vec3(T(-1), T(-1), T(1)).asDiagonal();

    EXPECT_EQ(mrpToDcm(mrp180AboutX), dcm180AboutX);
    EXPECT_EQ(mrpToDcm(mrp180AboutY), dcm180AboutY);
    EXPECT_EQ(mrpToDcm(mrp180AboutZ), dcm180AboutZ);
}

// Verify that mrpToDcm preserves a very small rotation about the Z-axis.
// At 1e-8 radians, cos(angle) rounds to 1 in float and double, so the rotation is
// carried only by the off-diagonal DCM elements.
TYPED_TEST(DcmToRepresentationTest, mrpToDcmSmallRotation) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;
    using Mat3 = Eigen::Matrix3<T>;

    T const angle = T(1e-8);
    Vec3 const mrp = Vec3(T(0), T(0), T(1)) * std::tan(angle / T(4));

    // DCM for a small rotation about Z.
    Mat3 expected;
    expected << std::cos(angle), std::sin(angle), T(0), -std::sin(angle), std::cos(angle), T(0), T(0), T(0), T(1);

    // Compare the deviation from the identity, so the small rotation is checked relative to its own size.
    Mat3 const identity = Mat3::Identity();
    EXPECT_TRUE((mrpToDcm(mrp) - identity).isApprox(expected - identity, T(10) * std::numeric_limits<T>::epsilon()));
}

// ---------------------------------------------------------------------------
// dcmToEp: converts a DCM to unit Euler parameters (scalar first) with q0 >= 0.
// ---------------------------------------------------------------------------
TYPED_TEST(DcmToRepresentationTest, dcmToEpMatchesCReference) {
    using T = TypeParam;
    Eigen::Matrix<T, 3, 3> dcm = rotationMatrix(this->dist(this->generator), 3) *
                                 rotationMatrix(this->dist(this->generator), 2) *
                                 rotationMatrix(this->dist(this->generator), 1);

    T dcmArray[3][3];
    eigenMatrix3ToCArray(dcm, dcmArray);
    T expectedArray[4] = {};
    if constexpr (std::is_same_v<T, float>) {
        C2EP_float(dcmArray, expectedArray);
    } else {
        C2EP(dcmArray, expectedArray);
    }
    auto expected = Eigen::Map<Eigen::Matrix<T, 4, 1>>(expectedArray);

    auto const identity = TestFixture::Mat3::Identity();
    T const orthoTolerance = std::max(rotationParameterTolerance(dcm), this->accuracy);
    EXPECT_LT(relativeErrorNorm(dcm * dcm.transpose(), identity), orthoTolerance);

    T const valueTolerance = std::max(rotationParameterTolerance(expected), this->accuracy);
    EXPECT_LT(relativeErrorNorm(dcmToEp(dcm), expected), valueTolerance);
}

// The identity DCM gives the identity Euler parameters (1, 0, 0, 0).
TYPED_TEST(DcmToRepresentationTest, dcmToEpIdentity) {
    using T = TypeParam;
    using Vec4 = Eigen::Vector4<T>;
    using Mat3 = Eigen::Matrix3<T>;

    Mat3 const identity = Mat3::Identity();
    EXPECT_EQ(dcmToEp(identity), Vec4(T(1), T(0), T(0), T(0)));
}

// Verify DCM-to-Euler-parameter conversion for a 90-degree rotation about X.
TYPED_TEST(DcmToRepresentationTest, dcmToEp90DegX) {
    using T = TypeParam;
    using Vec4 = Eigen::Vector4<T>;
    using Mat3 = Eigen::Matrix3<T>;

    // DCM representing a 90-degree rotation about X.
    Mat3 dcm;
    dcm << T(1), T(0), T(0), T(0), T(0), T(1), T(0), T(-1), T(0);

    // Calculate the expected Euler parameters using q = (cos(phi / 2), sin(phi / 2) * axis).
    T const phi = T(90) * std::numbers::pi_v<T> / T(180);
    Vec4 const expected(std::cos(phi / T(2)), std::sin(phi / T(2)), T(0), T(0));

    EXPECT_TRUE(dcmToEp(dcm).isApprox(expected, T(10) * std::numeric_limits<T>::epsilon()));
}

// A 270-degree rotation about X gives q0 = cos(135 deg) < 0. dcmToEp returns q0 >= 0, so it must return
// the negated set, which describes the same rotation.
TYPED_TEST(DcmToRepresentationTest, dcmToEp270DegX) {
    using T = TypeParam;
    using Vec4 = Eigen::Vector4<T>;
    using Mat3 = Eigen::Matrix3<T>;

    // DCM representing a 270-degree rotation about X.
    Mat3 dcm;
    dcm << T(1), T(0), T(0), T(0), T(0), T(-1), T(0), T(1), T(0);

    // Calculate q = (cos(phi / 2), sin(phi / 2) * axis) and negate it so that q0 >= 0.
    T const phi = T(270) * std::numbers::pi_v<T> / T(180);
    Vec4 const expected = -Vec4(std::cos(phi / T(2)), std::sin(phi / T(2)), T(0), T(0));

    EXPECT_TRUE(dcmToEp(dcm).isApprox(expected, T(10) * std::numeric_limits<T>::epsilon()));
}

// Verify 180-degree rotations about each axis: q0 = 0 and the vector part is the rotation axis.
// Each case uses a different branch of Stanley's method. Either sign of the axis describes the same rotation.
TYPED_TEST(DcmToRepresentationTest, dcmToEp180Degrees) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;
    using Vec4 = Eigen::Vector4<T>;
    using Mat3 = Eigen::Matrix3<T>;

    Mat3 const dcm180AboutX = Vec3(T(1), T(-1), T(-1)).asDiagonal();
    Mat3 const dcm180AboutY = Vec3(T(-1), T(1), T(-1)).asDiagonal();
    Mat3 const dcm180AboutZ = Vec3(T(-1), T(-1), T(1)).asDiagonal();

    EXPECT_EQ(dcmToEp(dcm180AboutX).cwiseAbs(), Vec4(T(0), T(1), T(0), T(0)));
    EXPECT_EQ(dcmToEp(dcm180AboutY).cwiseAbs(), Vec4(T(0), T(0), T(1), T(0)));
    EXPECT_EQ(dcmToEp(dcm180AboutZ).cwiseAbs(), Vec4(T(0), T(0), T(0), T(1)));
}

// Verify 150-degree rotations whose largest Euler parameter is q1, q2 or q3, so each of those branches of Stanley's
// method runs with non-zero off-diagonal terms. The negative axis component in the q1 and q3 cases makes those
// branches flip the sign of the set to keep q0 >= 0; the all-positive q2 case needs no flip.
TYPED_TEST(DcmToRepresentationTest, dcmToEpLargeRotations) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;
    using Vec4 = Eigen::Vector4<T>;
    using Mat3 = Eigen::Matrix3<T>;

    T const phi = T(150) * std::numbers::pi_v<T> / T(180);
    Vec3 const axisForQ1(T(-0.8), T(0.48), T(0.36));
    Vec3 const axisForQ2(T(0.36), T(0.8), T(0.48));
    Vec3 const axisForQ3(T(0.48), T(0.36), T(-0.8));

    for (Vec3 const& axis : {axisForQ1, axisForQ2, axisForQ3}) {
        // DCM of the rotation, built independently with Eigen::AngleAxis (the DCM is the transpose of its matrix).
        Mat3 const dcm = Eigen::AngleAxis<T>(phi, axis).toRotationMatrix().transpose();

        // Calculate the expected Euler parameters using q = (cos(phi / 2), sin(phi / 2) * axis); q0 = cos(75 deg) > 0.
        Vec4 expected;
        expected << std::cos(phi / T(2)), std::sin(phi / T(2)) * axis;

        EXPECT_TRUE(dcmToEp(dcm).isApprox(expected, T(10) * std::numeric_limits<T>::epsilon()));
    }
}

// Verify that dcmToEp preserves a very small rotation about the Z-axis.
// At 1e-8 radians, cos(angle) rounds to 1 in float and double, so the rotation is
// carried only by the off-diagonal DCM elements.
TYPED_TEST(DcmToRepresentationTest, dcmToEpSmallRotation) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;
    using Vec4 = Eigen::Vector4<T>;
    using Mat3 = Eigen::Matrix3<T>;

    T const angle = T(1e-8);

    // Construct the DCM for a small rotation about Z.
    Mat3 dcm;
    dcm << std::cos(angle), std::sin(angle), T(0), -std::sin(angle), std::cos(angle), T(0), T(0), T(0), T(1);

    // Expected vector part: sin(angle / 2) * axis, about 5e-9 about Z.
    Vec3 const expectedVector(T(0), T(0), std::sin(angle / T(2)));

    // Compare the vector part relative to its own size, so the small rotation is not lost next to q0 = 1.
    Vec4 const ep = dcmToEp(dcm);
    EXPECT_TRUE(ep.template tail<3>().isApprox(expectedVector, T(10) * std::numeric_limits<T>::epsilon()));
}

// ---------------------------------------------------------------------------
// dcmToPrv: converts a DCM to the principal rotation vector of the shortest rotation, angle in [0, pi].
// ---------------------------------------------------------------------------
TYPED_TEST(DcmToRepresentationTest, dcmToPrvMatchesCReference) {
    using T = TypeParam;
    Eigen::Matrix<T, 3, 3> dcm = rotationMatrix(this->dist(this->generator), 3) *
                                 rotationMatrix(this->dist(this->generator), 2) *
                                 rotationMatrix(this->dist(this->generator), 1);

    T dcmArray[3][3];
    eigenMatrix3ToCArray(dcm, dcmArray);
    T expectedArray[3] = {};
    if constexpr (std::is_same_v<T, float>) {
        C2PRV_float(dcmArray, expectedArray);
    } else {
        C2PRV(dcmArray, expectedArray);
    }
    auto expected = Eigen::Map<Eigen::Matrix<T, 3, 1>>(expectedArray);

    auto const identity = TestFixture::Mat3::Identity();
    T const orthoTolerance = std::max(rotationParameterTolerance(dcm), this->accuracy);
    EXPECT_LT(relativeErrorNorm(dcm * dcm.transpose(), identity), orthoTolerance);

    T const valueTolerance = std::max(rotationParameterTolerance(expected), this->accuracy);
    EXPECT_LT(relativeErrorNorm(dcmToPrv(dcm), expected), valueTolerance);
}

// The identity DCM gives the zero principal rotation vector.
TYPED_TEST(DcmToRepresentationTest, dcmToPrvIdentityIsZero) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;
    using Mat3 = Eigen::Matrix3<T>;

    Mat3 const identity = Mat3::Identity();
    EXPECT_EQ(dcmToPrv(identity), Vec3::Zero());
}

// Verify DCM-to-PRV conversion for a 90-degree rotation about X.
TYPED_TEST(DcmToRepresentationTest, dcmToPrv90DegX) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;
    using Mat3 = Eigen::Matrix3<T>;

    // DCM representing a 90-degree rotation about X.
    Mat3 dcm;
    dcm << T(1), T(0), T(0), T(0), T(0), T(1), T(0), T(-1), T(0);

    // Calculate the expected PRV using prv = phi * axis.
    Vec3 const axis(T(1), T(0), T(0));
    T const phi = T(90) * std::numbers::pi_v<T> / T(180);

    EXPECT_TRUE(dcmToPrv(dcm).isApprox(phi * axis, T(10) * std::numeric_limits<T>::epsilon()));
}

// A 270-degree rotation about X is equivalent to a -90-degree rotation about X.
// dcmToPrv must return the shortest rotation, with an angle of at most 180 degrees.
TYPED_TEST(DcmToRepresentationTest, dcmToPrv270DegX) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;
    using Mat3 = Eigen::Matrix3<T>;

    Mat3 dcm270AboutX;
    dcm270AboutX << T(1), T(0), T(0), T(0), T(0), T(-1), T(0), T(1), T(0);

    // The equivalent shortest rotation is 90 degrees about the negative X-axis.
    Vec3 const principalAxis(T(-1), T(0), T(0));
    T const principalAngle = T(90) * std::numbers::pi_v<T> / T(180);

    EXPECT_TRUE(
        dcmToPrv(dcm270AboutX).isApprox(principalAngle * principalAxis, T(10) * std::numeric_limits<T>::epsilon()));
}

// Verify 180-degree rotations about each axis. Either sign of the axis describes the same rotation.
TYPED_TEST(DcmToRepresentationTest, dcmToPrv180Degrees) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;
    using Mat3 = Eigen::Matrix3<T>;

    T const tol = T(10) * std::numeric_limits<T>::epsilon();
    T const pi = std::numbers::pi_v<T>;

    Mat3 const dcm180AboutX = Vec3(T(1), T(-1), T(-1)).asDiagonal();
    Mat3 const dcm180AboutY = Vec3(T(-1), T(1), T(-1)).asDiagonal();
    Mat3 const dcm180AboutZ = Vec3(T(-1), T(-1), T(1)).asDiagonal();

    // At 180 degrees, prv = pi * axis.
    EXPECT_TRUE(dcmToPrv(dcm180AboutX).cwiseAbs().isApprox(pi * Vec3::UnitX(), tol));
    EXPECT_TRUE(dcmToPrv(dcm180AboutY).cwiseAbs().isApprox(pi * Vec3::UnitY(), tol));
    EXPECT_TRUE(dcmToPrv(dcm180AboutZ).cwiseAbs().isApprox(pi * Vec3::UnitZ(), tol));
}

// Verify that dcmToPrv preserves a very small rotation about the Z-axis.
// At 1e-8 radians, cos(angle) rounds to 1 in float and double, so the
// rotation is carried only by the off-diagonal DCM elements.
TYPED_TEST(DcmToRepresentationTest, dcmToPrvSmallRotation) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;
    using Mat3 = Eigen::Matrix3<T>;

    T const angle = T(1e-8);

    // Construct the DCM for a small rotation about Z.
    Mat3 dcm;
    dcm << std::cos(angle), std::sin(angle), T(0), -std::sin(angle), std::cos(angle), T(0), T(0), T(0), T(1);

    // Calculate the expected PRV using prv = angle * axis.
    Vec3 const expected(T(0), T(0), angle);

    EXPECT_TRUE(dcmToPrv(dcm).isApprox(expected, T(10) * std::numeric_limits<T>::epsilon()));
}

// ---------------------------------------------------------------------------
// prvToDcm: converts a principal rotation vector to its direction cosine matrix.
// ---------------------------------------------------------------------------
TYPED_TEST(DcmToRepresentationTest, prvToDcmMatchesCReference) {
    using T = TypeParam;
    using Vec3 = Eigen::Matrix<T, 3, 1>;
    using Mat3 = Eigen::Matrix<T, 3, 3>;

    Vec3 prv(this->dist(this->generator), this->dist(this->generator), this->dist(this->generator));
    T expectedArray[3][3] = {};
    if constexpr (std::is_same_v<T, float>) {
        PRV2C_float(prv.data(), expectedArray);
    } else {
        PRV2C(prv.data(), expectedArray);
    }

    Mat3 expected = cArray33ToEigenMatrix33(expectedArray);
    Mat3 result = prvToDcm<T>(prv);

    auto const identity = Mat3::Identity();
    T const orthoTolerance = std::max(rotationParameterTolerance(result), this->accuracy);
    EXPECT_LT(relativeErrorNorm(result * result.transpose(), identity), orthoTolerance);

    T const valueTolerance = std::max(rotationParameterTolerance(expected), this->accuracy);
    EXPECT_LT(relativeErrorNorm(result, expected), valueTolerance);
}

// The zero principal rotation vector gives the identity DCM.
TYPED_TEST(DcmToRepresentationTest, prvToDcmZeroIsIdentity) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;
    using Mat3 = Eigen::Matrix3<T>;

    Vec3 const zeroPrv = Vec3::Zero();
    Mat3 const identity = Mat3::Identity();
    EXPECT_EQ(prvToDcm(zeroPrv), identity);
}

// Verify PRV-to-DCM conversion for a 90-degree rotation about X.
TYPED_TEST(DcmToRepresentationTest, prvToDcm90DegX) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;
    using Mat3 = Eigen::Matrix3<T>;

    // Calculate the PRV using prv = phi * axis.
    Vec3 const axis(T(1), T(0), T(0));
    T const phi = T(90) * std::numbers::pi_v<T> / T(180);
    Vec3 const prv = phi * axis;

    // DCM representing a 90-degree rotation about X.
    Mat3 expected;
    expected << T(1), T(0), T(0), T(0), T(0), T(1), T(0), T(-1), T(0);

    EXPECT_TRUE(prvToDcm(prv).isApprox(expected, T(10) * std::numeric_limits<T>::epsilon()));
}

// A 270-degree rotation about X gives the same DCM as the -90-degree rotation. prvToDcm accepts any angle.
TYPED_TEST(DcmToRepresentationTest, prvToDcm270DegX) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;
    using Mat3 = Eigen::Matrix3<T>;

    // Calculate the PRV using prv = phi * axis.
    Vec3 const axis(T(1), T(0), T(0));
    T const phi = T(270) * std::numbers::pi_v<T> / T(180);
    Vec3 const prv = phi * axis;

    // DCM representing a 270-degree (equivalently -90-degree) rotation about X.
    Mat3 expected;
    expected << T(1), T(0), T(0), T(0), T(0), T(-1), T(0), T(1), T(0);

    EXPECT_TRUE(prvToDcm(prv).isApprox(expected, T(10) * std::numeric_limits<T>::epsilon()));
}

// Verify 180-degree rotations about each principal axis.
TYPED_TEST(DcmToRepresentationTest, prvToDcm180Degrees) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;
    using Mat3 = Eigen::Matrix3<T>;

    T const tol = T(10) * std::numeric_limits<T>::epsilon();
    T const pi = std::numbers::pi_v<T>;

    Vec3 const prv180AboutX = pi * Vec3::UnitX();
    Vec3 const prv180AboutY = pi * Vec3::UnitY();
    Vec3 const prv180AboutZ = pi * Vec3::UnitZ();

    Mat3 const dcm180AboutX = Vec3(T(1), T(-1), T(-1)).asDiagonal();
    Mat3 const dcm180AboutY = Vec3(T(-1), T(1), T(-1)).asDiagonal();
    Mat3 const dcm180AboutZ = Vec3(T(-1), T(-1), T(1)).asDiagonal();

    // sin(pi) is not exactly 0 in floating point, so the off-diagonal terms are compared within tolerance.
    EXPECT_TRUE(prvToDcm(prv180AboutX).isApprox(dcm180AboutX, tol));
    EXPECT_TRUE(prvToDcm(prv180AboutY).isApprox(dcm180AboutY, tol));
    EXPECT_TRUE(prvToDcm(prv180AboutZ).isApprox(dcm180AboutZ, tol));
}

// A full 360-degree turn about any axis gives the identity DCM.
TYPED_TEST(DcmToRepresentationTest, prvToDcm360DegreesIsIdentity) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;
    using Mat3 = Eigen::Matrix3<T>;

    Vec3 const axis(T(0.6), T(0), T(0.8));
    Vec3 const prv = T(2) * std::numbers::pi_v<T> * axis;

    EXPECT_TRUE(prvToDcm(prv).isApprox(Mat3::Identity(), T(10) * std::numeric_limits<T>::epsilon()));
}

// Verify that prvToDcm preserves a very small rotation about the Z-axis.
// At 1e-8 radians, cos(angle) rounds to 1 in float and double, so the rotation is
// carried only by the off-diagonal DCM elements.
TYPED_TEST(DcmToRepresentationTest, prvToDcmSmallRotation) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;
    using Mat3 = Eigen::Matrix3<T>;

    T const angle = T(1e-8);
    Vec3 const prv(T(0), T(0), angle);

    // Construct the DCM for a small rotation about Z.
    Mat3 expected;
    expected << std::cos(angle), std::sin(angle), T(0), -std::sin(angle), std::cos(angle), T(0), T(0), T(0), T(1);

    // Compare the deviation from the identity, so the small rotation is checked relative to its own size.
    Mat3 const identity = Mat3::Identity();
    EXPECT_TRUE((prvToDcm(prv) - identity).isApprox(expected - identity, T(10) * std::numeric_limits<T>::epsilon()));
}

TYPED_TEST(DcmToRepresentationTest, dcmToEuler321) {
    using T = TypeParam;
    Eigen::Matrix<T, 3, 3> dcm = rotationMatrix(this->dist(this->generator), 3) *
                                 rotationMatrix(this->dist(this->generator), 2) *
                                 rotationMatrix(this->dist(this->generator), 1);

    T dcmArray[3][3];
    eigenMatrix3ToCArray(dcm, dcmArray);
    T expectedArray[3] = {};
    if constexpr (std::is_same_v<T, float>) {
        C2Euler321_float(dcmArray, expectedArray);
    } else {
        C2Euler321(dcmArray, expectedArray);
    }
    auto expected = Eigen::Map<Eigen::Matrix<T, 3, 1>>(expectedArray);

    auto const identity = TestFixture::Mat3::Identity();
    T const orthoTolerance = std::max(rotationParameterTolerance(dcm), this->accuracy);
    EXPECT_LT(relativeErrorNorm(dcm * dcm.transpose(), identity), orthoTolerance);

    T const valueTolerance = std::max(rotationParameterTolerance(expected), this->accuracy);
    EXPECT_LT(relativeErrorNorm(dcmToEulerAngles321(dcm), expected), valueTolerance);
}

TYPED_TEST(DcmToRepresentationTest, eulerAngles321ToDcm) {
    using T = TypeParam;
    using Vec3 = Eigen::Matrix<T, 3, 1>;
    using Mat3 = Eigen::Matrix<T, 3, 3>;

    Vec3 euler(this->dist(this->generator), this->dist(this->generator), this->dist(this->generator));
    T expectedArray[3][3] = {};
    if constexpr (std::is_same_v<T, float>) {
        Euler3212C_float(euler.data(), expectedArray);
    } else {
        Euler3212C(euler.data(), expectedArray);
    }

    Mat3 expected = cArray33ToEigenMatrix33(expectedArray);
    Mat3 result = eulerAngles321ToDcm<T>(euler);

    auto const identity = Mat3::Identity();
    T const orthoTolerance = std::max(rotationParameterTolerance(result), this->accuracy);
    EXPECT_LT(relativeErrorNorm(result * result.transpose(), identity), orthoTolerance);

    T const valueTolerance = std::max(rotationParameterTolerance(expected), this->accuracy);
    EXPECT_LT(relativeErrorNorm(result, expected), valueTolerance);
}

TYPED_TEST(DcmToRepresentationTest, eulerParameterToDcm) {
    using T = TypeParam;
    using Vec4 = Eigen::Matrix<T, 4, 1>;
    using Mat3 = Eigen::Matrix<T, 3, 3>;

    T phi = this->dist(this->generator);
    Eigen::Matrix<T, 3, 1> unitVector(
        this->dist(this->generator), this->dist(this->generator), this->dist(this->generator));
    unitVector.normalize();
    Vec4 ep(std::cos(phi / 2),
            std::sin(phi / 2) * unitVector(0),
            std::sin(phi / 2) * unitVector(1),
            std::sin(phi / 2) * unitVector(2));

    T expectedArray[3][3] = {};
    if constexpr (std::is_same_v<T, float>) {
        EP2C_float(ep.data(), expectedArray);
    } else {
        EP2C(ep.data(), expectedArray);
    }

    Mat3 expected = cArray33ToEigenMatrix33(expectedArray);
    Mat3 result = epToDcm(ep);

    auto const identity = Mat3::Identity();
    T const orthoTolerance = std::max(rotationParameterTolerance(result), this->accuracy);
    EXPECT_LT(relativeErrorNorm(result * result.transpose(), identity), orthoTolerance);

    T const valueTolerance = std::max(rotationParameterTolerance(expected), this->accuracy);
    EXPECT_LT(relativeErrorNorm(result, expected), valueTolerance);
}

// Representation-to-representation tests
template <typename T>
class RepresentationTransformTest : public ::testing::Test {
   public:
    using Vec3 = Eigen::Matrix<T, 3, 1>;

    std::default_random_engine generator{std::random_device{}()};
    std::uniform_real_distribution<T> dist{-std::numbers::pi, std::numbers::pi};
    T const accuracy = kinematicsAccuracy<T>();
};

TYPED_TEST_SUITE(RepresentationTransformTest, FloatingPointTypes);

// ---------------------------------------------------------------------------
// epToMrp: converts Euler parameters to the principal MRP set.
// ---------------------------------------------------------------------------
TYPED_TEST(RepresentationTransformTest, epToMrpMatchesCReference) {
    using T = TypeParam;
    using Vec4 = Eigen::Matrix<T, 4, 1>;
    using Vec3 = Eigen::Matrix<T, 3, 1>;

    T phi = TestFixture::dist(TestFixture::generator);
    Vec3 axis(TestFixture::dist(TestFixture::generator),
              TestFixture::dist(TestFixture::generator),
              TestFixture::dist(TestFixture::generator));
    axis.normalize();
    Vec4 ep(std::cos(phi / 2), std::sin(phi / 2) * axis(0), std::sin(phi / 2) * axis(1), std::sin(phi / 2) * axis(2));

    T expectedArray[3] = {};
    if constexpr (std::is_same_v<T, float>) {
        EP2MRP_float(ep.data(), expectedArray);
    } else {
        EP2MRP(ep.data(), expectedArray);
    }

    Vec3 expected = Eigen::Map<Vec3>(expectedArray);
    Vec3 result = epToMrp<T>(ep);

    T const tolerance = std::max(rotationParameterTolerance(expected), TestFixture::accuracy);
    EXPECT_LT(relativeErrorNorm(result, expected), tolerance);
}

// The identity Euler parameters (1, 0, 0, 0) give the zero MRP set.
TYPED_TEST(RepresentationTransformTest, epToMrpIdentityIsZero) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;
    using Vec4 = Eigen::Vector4<T>;

    Vec4 const identity(T(1), T(0), T(0), T(0));
    EXPECT_EQ(epToMrp(identity), Vec3::Zero());
}

// Verify EP-to-MRP conversion for a 90-degree rotation about X, given as q and as -q.
TYPED_TEST(RepresentationTransformTest, epToMrp90DegX) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;
    using Vec4 = Eigen::Vector4<T>;

    T const tol = T(10) * std::numeric_limits<T>::epsilon();

    // Euler parameters of a 90-degree rotation about X: q = (cos(phi / 2), sin(phi / 2) * axis).
    Vec3 const axis(T(1), T(0), T(0));
    T const phi = T(90) * std::numbers::pi_v<T> / T(180);
    Vec4 ep;
    ep << std::cos(phi / T(2)), std::sin(phi / T(2)) * axis;

    // Calculate the expected MRP using sigma = axis * tan(phi / 4).
    Vec3 const expected = axis * std::tan(phi / T(4));

    // Verify the converted MRP against the analytical result.
    EXPECT_TRUE(epToMrp(ep).isApprox(expected, tol));

    // -q describes the same rotation, so it must give the same MRP set.
    Vec4 const negatedEp = -ep;
    EXPECT_TRUE(epToMrp(negatedEp).isApprox(expected, tol));
}

// A 270-degree rotation about X gives q0 = cos(135 deg) < 0. It is equivalent to a -90-degree rotation
// about X, and epToMrp must return that principal (shortest-rotation) MRP set, whose norm is at most 1.
TYPED_TEST(RepresentationTransformTest, epToMrp270DegX) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;
    using Vec4 = Eigen::Vector4<T>;

    // Euler parameters of a 270-degree rotation about X: q = (cos(phi / 2), sin(phi / 2) * axis).
    T const phi = T(270) * std::numbers::pi_v<T> / T(180);
    Vec4 const ep270AboutX(std::cos(phi / T(2)), std::sin(phi / T(2)), T(0), T(0));

    // Calculate the expected MRP using sigma = axis * tan(phi / 4).
    // The equivalent shortest rotation is 90 degrees about the negative X-axis.
    Vec3 const principalAxis(T(-1), T(0), T(0));
    T const principalAngle = T(90) * std::numbers::pi_v<T> / T(180);
    Vec3 const expectedMrp = principalAxis * std::tan(principalAngle / T(4));

    // Verify that the 270-degree Euler parameters produce the MRP of the equivalent -90-degree rotation.
    EXPECT_TRUE(epToMrp(ep270AboutX).isApprox(expectedMrp, T(10) * std::numeric_limits<T>::epsilon()));
}

// Verify 180-degree rotations about each axis: q0 = 0 and the MRP set equals the rotation axis.
TYPED_TEST(RepresentationTransformTest, epToMrp180Degrees) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;
    using Vec4 = Eigen::Vector4<T>;

    Vec4 const ep180AboutX(T(0), T(1), T(0), T(0));
    Vec4 const ep180AboutY(T(0), T(0), T(1), T(0));
    Vec4 const ep180AboutZ(T(0), T(0), T(0), T(1));

    // At 180 degrees, sigma = axis * tan(phi / 4) = axis.
    EXPECT_EQ(epToMrp(ep180AboutX), Vec3::UnitX());
    EXPECT_EQ(epToMrp(ep180AboutY), Vec3::UnitY());
    EXPECT_EQ(epToMrp(ep180AboutZ), Vec3::UnitZ());
}

// Verify that epToMrp preserves a very small rotation about the Z-axis.
// At 1e-8 radians, q0 = cos(angle / 2) rounds to 1 in float and double, so the
// rotation is carried only by the vector part of the Euler parameters.
TYPED_TEST(RepresentationTransformTest, epToMrpSmallRotation) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;
    using Vec4 = Eigen::Vector4<T>;

    T const angle = T(1e-8);
    Vec4 const ep(std::cos(angle / T(2)), T(0), T(0), std::sin(angle / T(2)));

    // Calculate the expected MRP using sigma = axis * tan(angle / 4), about 2.5e-9 about Z.
    Vec3 const expected(T(0), T(0), std::tan(angle / T(4)));

    EXPECT_TRUE(epToMrp(ep).isApprox(expected, T(10) * std::numeric_limits<T>::epsilon()));
}

// An all-zero quaternion is not a valid rotation. epToMrp returns the zero MRP set (identity) without an error.
TYPED_TEST(RepresentationTransformTest, epToMrpOfZeroIsZero) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;
    using Vec4 = Eigen::Vector4<T>;

    EXPECT_EQ(epToMrp(Vec4(Vec4::Zero())), Vec3::Zero());
}

// ---------------------------------------------------------------------------
// epToPrv: converts Euler parameters to the principal rotation vector, keeping the sign of q0.
// ---------------------------------------------------------------------------
TYPED_TEST(RepresentationTransformTest, epToPrvMatchesCReference) {
    using T = TypeParam;
    using Vec4 = Eigen::Matrix<T, 4, 1>;
    using Vec3 = Eigen::Matrix<T, 3, 1>;

    T phi = TestFixture::dist(TestFixture::generator);
    Vec3 axis(TestFixture::dist(TestFixture::generator),
              TestFixture::dist(TestFixture::generator),
              TestFixture::dist(TestFixture::generator));
    axis.normalize();
    Vec4 ep(std::cos(phi / 2), std::sin(phi / 2) * axis(0), std::sin(phi / 2) * axis(1), std::sin(phi / 2) * axis(2));

    T expectedArray[3] = {};
    if constexpr (std::is_same_v<T, float>) {
        EP2PRV_float(ep.data(), expectedArray);
    } else {
        EP2PRV(ep.data(), expectedArray);
    }

    Vec3 expected = Eigen::Map<Vec3>(expectedArray);
    Vec3 result = epToPrv<T>(ep);

    T const tolerance = std::max(rotationParameterTolerance(expected), TestFixture::accuracy);
    EXPECT_LT(relativeErrorNorm(result, expected), tolerance);
}

// The identity Euler parameters (1, 0, 0, 0) give the zero principal rotation vector.
TYPED_TEST(RepresentationTransformTest, epToPrvIdentityIsZero) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;
    using Vec4 = Eigen::Vector4<T>;

    Vec4 const identity(T(1), T(0), T(0), T(0));
    EXPECT_EQ(epToPrv(identity), Vec3::Zero());
}

// Verify EP-to-PRV conversion for a 90-degree rotation about X, given as q and as -q.
TYPED_TEST(RepresentationTransformTest, epToPrv90DegX) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;
    using Vec4 = Eigen::Vector4<T>;

    T const tol = T(10) * std::numeric_limits<T>::epsilon();

    // Euler parameters of a 90-degree rotation about X: q = (cos(phi / 2), sin(phi / 2) * axis).
    Vec3 const axis(T(1), T(0), T(0));
    T const phi = T(90) * std::numbers::pi_v<T> / T(180);
    Vec4 ep;
    ep << std::cos(phi / T(2)), std::sin(phi / T(2)) * axis;

    // Calculate the expected principal rotation vector using prv = phi * axis.
    EXPECT_TRUE(epToPrv(ep).isApprox(phi * axis, tol));

    // epToPrv keeps the sign of q0, so -q gives the long rotation: 360 - 90 = 270 degrees about -X.
    Vec4 const negatedEp = -ep;
    T const longPhi = T(270) * std::numbers::pi_v<T> / T(180);
    EXPECT_TRUE(epToPrv(negatedEp).isApprox(longPhi * -axis, tol));
}

// A 270-degree rotation about X gives q0 = cos(135 deg) < 0. epToPrv does not switch to the short
// rotation, so the result is the 270-degree rotation vector itself.
TYPED_TEST(RepresentationTransformTest, epToPrv270DegX) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;
    using Vec4 = Eigen::Vector4<T>;

    // Euler parameters of a 270-degree rotation about X: q = (cos(phi / 2), sin(phi / 2) * axis).
    Vec3 const axis(T(1), T(0), T(0));
    T const phi = T(270) * std::numbers::pi_v<T> / T(180);
    Vec4 ep270AboutX;
    ep270AboutX << std::cos(phi / T(2)), std::sin(phi / T(2)) * axis;

    EXPECT_TRUE(epToPrv(ep270AboutX).isApprox(phi * axis, T(10) * std::numeric_limits<T>::epsilon()));
}

// Verify 180-degree rotations about each axis: q0 = 0 and the rotation vector is pi times the axis.
TYPED_TEST(RepresentationTransformTest, epToPrv180Degrees) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;
    using Vec4 = Eigen::Vector4<T>;

    T const tol = T(10) * std::numeric_limits<T>::epsilon();
    T const pi = std::numbers::pi_v<T>;

    Vec4 const ep180AboutX(T(0), T(1), T(0), T(0));
    Vec4 const ep180AboutY(T(0), T(0), T(1), T(0));
    Vec4 const ep180AboutZ(T(0), T(0), T(0), T(1));

    EXPECT_TRUE(epToPrv(ep180AboutX).isApprox(pi * Vec3::UnitX(), tol));
    EXPECT_TRUE(epToPrv(ep180AboutY).isApprox(pi * Vec3::UnitY(), tol));
    EXPECT_TRUE(epToPrv(ep180AboutZ).isApprox(pi * Vec3::UnitZ(), tol));
}

// Verify that epToPrv preserves a very small rotation about the Z-axis.
// At 1e-8 radians, q0 rounds to 1, so using acos(q0) would lose the rotation.
TYPED_TEST(RepresentationTransformTest, epToPrvSmallRotation) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;
    using Vec4 = Eigen::Vector4<T>;

    T const angle = T(1e-8);
    Vec3 const axis(T(0), T(0), T(1));

    // Construct the quaternion using q = (cos(angle / 2), sin(angle / 2) * axis).
    Vec4 ep;
    ep << std::cos(angle / T(2)), std::sin(angle / T(2)) * axis;

    // Calculate the expected PRV using prv = angle * axis.
    Vec3 const expected = angle * axis;

    EXPECT_TRUE(epToPrv(ep).isApprox(expected, T(10) * std::numeric_limits<T>::epsilon()));
}

// Verify that scaling the Euler parameters by a positive factor does not
// change the resulting PRV.
TYPED_TEST(RepresentationTransformTest, epToPrvNonUnitEp) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;
    using Vec4 = Eigen::Vector4<T>;

    Vec3 const axis(T(1), T(0), T(0));
    T const phi = T(90) * std::numbers::pi_v<T> / T(180);
    Vec4 ep;
    ep << std::cos(phi / T(2)), std::sin(phi / T(2)) * axis;

    Vec4 const scaledEp = T(2) * ep;

    // The scaled Euler parameters must preserve the original rotation.
    EXPECT_TRUE(epToPrv(scaledEp).isApprox(phi * axis, T(10) * std::numeric_limits<T>::epsilon()));
}

// A zero vector part gives the zero rotation vector: (-1, 0, 0, 0) is a 360-degree rotation, and the all-zero
// quaternion is not a valid rotation but returns zero without an error.
TYPED_TEST(RepresentationTransformTest, epToPrvZeroVectorPartIsZero) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;
    using Vec4 = Eigen::Vector4<T>;

    Vec4 const ep360(T(-1), T(0), T(0), T(0));
    EXPECT_EQ(epToPrv(ep360), Vec3::Zero());
    EXPECT_EQ(epToPrv(Vec4(Vec4::Zero())), Vec3::Zero());
}

// ---------------------------------------------------------------------------
// subMrp: computes the MRP set of frame 1 relative to frame 2.
// ---------------------------------------------------------------------------
// The relative rotation between two equal MRP sets is the zero MRP set, up to rounding.
TYPED_TEST(RepresentationTransformTest, subMrpOfSameIsZero) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;

    Vec3 const mrp(T(0.1), T(-0.2), T(0.3));
    EXPECT_TRUE(subMrp(mrp, mrp).isZero(T(10) * std::numeric_limits<T>::epsilon()));
}

// Subtracting the zero MRP set returns the MRP set itself; subtracting from the zero MRP set returns its inverse.
TYPED_TEST(RepresentationTransformTest, subMrpWithZero) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;

    Vec3 const mrp(T(0.1), T(-0.2), T(0.3));
    Vec3 const zeroMrp = Vec3::Zero();
    EXPECT_EQ(subMrp(mrp, zeroMrp), mrp);
    EXPECT_EQ(subMrp(zeroMrp, mrp), -mrp);
}

// For rotations about the same axis, the relative rotation angle is the difference of the angles: 90 - 30 = 60 deg.
TYPED_TEST(RepresentationTransformTest, subMrpSameAxis) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;

    T const degToRad = std::numbers::pi_v<T> / T(180);
    Vec3 const axis(T(0.48), T(0.36), T(-0.8));

    // Calculate the MRPs using sigma = axis * tan(phi / 4).
    Vec3 const mrp1 = std::tan(T(90) * degToRad / T(4)) * axis;
    Vec3 const mrp2 = std::tan(T(30) * degToRad / T(4)) * axis;
    Vec3 const expected = std::tan(T(60) * degToRad / T(4)) * axis;

    EXPECT_TRUE(subMrp(mrp1, mrp2).isApprox(expected, T(10) * std::numeric_limits<T>::epsilon()));
}

// A relative rotation above 180 deg is returned as its principal set: 150 - (-150) = 300 deg about X is
// the -60 deg rotation about X.
TYPED_TEST(RepresentationTransformTest, subMrpReturnsPrincipalSet) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;

    T const degToRad = std::numbers::pi_v<T> / T(180);
    Vec3 const axis(T(1), T(0), T(0));

    // Calculate the MRPs using sigma = axis * tan(phi / 4).
    Vec3 const mrp1 = std::tan(T(150) * degToRad / T(4)) * axis;
    Vec3 const mrp2 = std::tan(T(-150) * degToRad / T(4)) * axis;
    Vec3 const expected = std::tan(T(-60) * degToRad / T(4)) * axis;

    EXPECT_TRUE(subMrp(mrp1, mrp2).isApprox(expected, T(10) * std::numeric_limits<T>::epsilon()));
}

// The result is the MRP set of frame 1 relative to frame 2, so its DCM is [C1] [C2]^T. The DCMs are built
// independently with Eigen::AngleAxis (the DCM is the transpose of its matrix).
TYPED_TEST(RepresentationTransformTest, subMrpFrameOrder) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;
    using Mat3 = Eigen::Matrix3<T>;

    T const degToRad = std::numbers::pi_v<T> / T(180);
    Vec3 const axis1(T(1), T(0), T(0));
    Vec3 const axis2(T(0), T(0.6), T(0.8));
    T const phi1 = T(70) * degToRad;
    T const phi2 = T(50) * degToRad;

    Vec3 const mrp1 = std::tan(phi1 / T(4)) * axis1;
    Vec3 const mrp2 = std::tan(phi2 / T(4)) * axis2;
    Mat3 const dcm1 = Eigen::AngleAxis<T>(phi1, axis1).toRotationMatrix().transpose();
    Mat3 const dcm2 = Eigen::AngleAxis<T>(phi2, axis2).toRotationMatrix().transpose();

    EXPECT_TRUE(
        mrpToDcm(subMrp(mrp1, mrp2)).isApprox(dcm1 * dcm2.transpose(), T(10) * std::numeric_limits<T>::epsilon()));
}

// Verify that subMrp uses the shadow MRP set when the denominator is near zero.
// For mrp1 = X and mrp2 = -0.9 X, the original denominator is 0.01.
// The equivalent principal relative rotation has MRP -(0.1 / 1.9) * X.
TYPED_TEST(RepresentationTransformTest, subMrpShadowBranch) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;

    Vec3 const axis(T(1), T(0), T(0));
    Vec3 const mrp1 = axis;
    Vec3 const mrp2 = T(-0.9) * axis;

    Vec3 const expected = -(T(1) - T(0.9)) / (T(1) + T(0.9)) * axis;

    EXPECT_TRUE(subMrp(mrp1, mrp2).isApprox(expected, T(10) * std::numeric_limits<T>::epsilon()));
}

// ---------------------------------------------------------------------------
// addMrp: computes the MRP set of two successive rotations, first mrp1, then mrp2.
// ---------------------------------------------------------------------------
// Adding the inverse rotation gives the zero MRP set, up to rounding.
TYPED_TEST(RepresentationTransformTest, addMrpInverseIsZero) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;

    Vec3 const mrp(T(0.1), T(-0.2), T(0.3));
    Vec3 const inverseMrp = -mrp;
    EXPECT_TRUE(addMrp(mrp, inverseMrp).isZero(T(10) * std::numeric_limits<T>::epsilon()));
}

// Adding the zero MRP set before or after a rotation returns the MRP set itself.
TYPED_TEST(RepresentationTransformTest, addMrpWithZero) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;

    Vec3 const mrp(T(0.1), T(-0.2), T(0.3));
    Vec3 const zeroMrp = Vec3::Zero();
    EXPECT_EQ(addMrp(mrp, zeroMrp), mrp);
    EXPECT_EQ(addMrp(zeroMrp, mrp), mrp);
}

// For rotations about the same axis, the angles add: 30 + 60 = 90 deg.
TYPED_TEST(RepresentationTransformTest, addMrpSameAxis) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;

    T const degToRad = std::numbers::pi_v<T> / T(180);
    Vec3 const axis(T(0.48), T(0.36), T(-0.8));

    // Calculate the MRPs using sigma = axis * tan(phi / 4).
    Vec3 const mrp1 = std::tan(T(30) * degToRad / T(4)) * axis;
    Vec3 const mrp2 = std::tan(T(60) * degToRad / T(4)) * axis;
    Vec3 const expected = std::tan(T(90) * degToRad / T(4)) * axis;

    EXPECT_TRUE(addMrp(mrp1, mrp2).isApprox(expected, T(10) * std::numeric_limits<T>::epsilon()));
}

// A total rotation above 180 deg is returned as its principal set: 150 + 150 = 300 deg about X is
// the -60 deg rotation about X.
TYPED_TEST(RepresentationTransformTest, addMrpReturnsPrincipalSet) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;

    T const degToRad = std::numbers::pi_v<T> / T(180);
    Vec3 const axis(T(1), T(0), T(0));

    // Calculate the MRPs using sigma = axis * tan(phi / 4).
    Vec3 const mrp = std::tan(T(150) * degToRad / T(4)) * axis;
    Vec3 const expected = std::tan(T(-60) * degToRad / T(4)) * axis;

    EXPECT_TRUE(addMrp(mrp, mrp).isApprox(expected, T(10) * std::numeric_limits<T>::epsilon()));
}

// The first rotation is mrp1 and the second is mrp2 relative to the frame mrp1 reaches, so the DCM of the result is
// [C2] [C1]. The DCMs are built independently with Eigen::AngleAxis (the DCM is the transpose of its matrix).
TYPED_TEST(RepresentationTransformTest, addMrpFrameOrder) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;
    using Mat3 = Eigen::Matrix3<T>;

    T const degToRad = std::numbers::pi_v<T> / T(180);
    Vec3 const axis1(T(1), T(0), T(0));
    Vec3 const axis2(T(0), T(0.6), T(0.8));
    T const phi1 = T(70) * degToRad;
    T const phi2 = T(50) * degToRad;

    Vec3 const mrp1 = std::tan(phi1 / T(4)) * axis1;
    Vec3 const mrp2 = std::tan(phi2 / T(4)) * axis2;
    Mat3 const dcm1 = Eigen::AngleAxis<T>(phi1, axis1).toRotationMatrix().transpose();
    Mat3 const dcm2 = Eigen::AngleAxis<T>(phi2, axis2).toRotationMatrix().transpose();

    EXPECT_TRUE(mrpToDcm(addMrp(mrp1, mrp2)).isApprox(dcm2 * dcm1, T(10) * std::numeric_limits<T>::epsilon()));
}

// Verify that addMrp uses the shadow MRP set when the denominator is near zero.
// For mrp1 = X and mrp2 = 0.9 X, the original denominator is 0.01.
// The equivalent principal total rotation has MRP -(0.1 / 1.9) * X.
TYPED_TEST(RepresentationTransformTest, addMrpShadowBranch) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;

    Vec3 const axis(T(1), T(0), T(0));
    Vec3 const mrp1 = axis;
    Vec3 const mrp2 = T(0.9) * axis;

    Vec3 const expected = -(T(1) - T(0.9)) / (T(1) + T(0.9)) * axis;

    EXPECT_TRUE(addMrp(mrp1, mrp2).isApprox(expected, T(10) * std::numeric_limits<T>::epsilon()));
}

// ---------------------------------------------------------------------------
// addPrv: computes the PRV of two successive rotations, first prv1, then prv2.
// ---------------------------------------------------------------------------
// Adding the zero PRV before or after a rotation returns the PRV itself.
TYPED_TEST(RepresentationTransformTest, addPrvWithZero) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;

    Vec3 const prv(T(0.1), T(-0.2), T(0.3));
    Vec3 const zeroPrv = Vec3::Zero();
    EXPECT_EQ(addPrv(prv, zeroPrv), prv);
    EXPECT_EQ(addPrv(zeroPrv, prv), prv);
}

// Adding the inverse rotation gives the zero PRV, up to rounding.
TYPED_TEST(RepresentationTransformTest, addPrvInverseIsZero) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;

    Vec3 const prv(T(0.1), T(-0.2), T(0.3));
    Vec3 const inversePrv = -prv;
    EXPECT_TRUE(addPrv(prv, inversePrv).isZero(T(10) * std::numeric_limits<T>::epsilon()));
}

// For rotations about the same axis, the angles add: 150 + 150 = 300 deg. The result is not switched to the
// short rotation (-60 deg).
TYPED_TEST(RepresentationTransformTest, addPrvSameAxis) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;

    T const degToRad = std::numbers::pi_v<T> / T(180);
    Vec3 const axis(T(1), T(0), T(0));
    Vec3 const prv = T(150) * degToRad * axis;
    Vec3 const expected = T(300) * degToRad * axis;

    EXPECT_TRUE(addPrv(prv, prv).isApprox(expected, T(10) * std::numeric_limits<T>::epsilon()));
}

// The first rotation is prv1 and the second is prv2 relative to the frame prv1 reaches, so the DCM of the result is
// [C2] [C1]. The DCMs are built independently with Eigen::AngleAxis (the DCM is the transpose of its matrix).
TYPED_TEST(RepresentationTransformTest, addPrvFrameOrder) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;
    using Mat3 = Eigen::Matrix3<T>;

    T const degToRad = std::numbers::pi_v<T> / T(180);
    Vec3 const axis1(T(1), T(0), T(0));
    Vec3 const axis2(T(0), T(0.6), T(0.8));
    T const phi1 = T(70) * degToRad;
    T const phi2 = T(50) * degToRad;

    Vec3 const prv1 = phi1 * axis1;
    Vec3 const prv2 = phi2 * axis2;
    Mat3 const dcm1 = Eigen::AngleAxis<T>(phi1, axis1).toRotationMatrix().transpose();
    Mat3 const dcm2 = Eigen::AngleAxis<T>(phi2, axis2).toRotationMatrix().transpose();

    EXPECT_TRUE(prvToDcm(addPrv(prv1, prv2)).isApprox(dcm2 * dcm1, T(10) * std::numeric_limits<T>::epsilon()));
}

// Small rotations must not be lost. Double only: in float these rotations are too small to see.
//  - A 1e-9 rad first rotation still changes the result. A shortcut that returns prv1 + prv2 would be off by
//    1/2 (prv1 x prv2), about 5e-10 rad here.
//  - Turning 1 rad and then back by (1 - 1e-8) rad leaves 1e-8 rad. A formula using 2*acos(cos(angle/2)) loses it.
TEST(RigidBodyKinematics, addPrvSmallRotations) {
    using Mat3 = Eigen::Matrix3d;

    Eigen::Vector3d const xAxis(1.0, 0.0, 0.0);
    Eigen::Vector3d const yAxis(0.0, 1.0, 0.0);
    double const tol = 10.0 * std::numeric_limits<double>::epsilon();

    double const tinyAngle = 1e-9;
    Mat3 const tinyDcm = Eigen::AngleAxisd(tinyAngle, xAxis).toRotationMatrix().transpose();
    Mat3 const yDcm = Eigen::AngleAxisd(1.0, yAxis).toRotationMatrix().transpose();
    EXPECT_TRUE(prvToDcm(addPrv(Eigen::Vector3d(tinyAngle * xAxis), yAxis)).isApprox(yDcm * tinyDcm, tol));

    double const netAngle = 1e-8;
    Mat3 const netDcm = Eigen::AngleAxisd(netAngle, xAxis).toRotationMatrix().transpose();
    EXPECT_TRUE(prvToDcm(addPrv(xAxis, Eigen::Vector3d(-(1.0 - netAngle) * xAxis))).isApprox(netDcm, tol));
}

// ---------------------------------------------------------------------------
// subPrv: computes the PRV of frame 1 relative to frame 2.
// ---------------------------------------------------------------------------
// Subtracting the zero PRV returns the PRV itself; subtracting from the zero PRV returns its inverse.
TYPED_TEST(RepresentationTransformTest, subPrvWithZero) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;

    Vec3 const prv(T(0.1), T(-0.2), T(0.3));
    Vec3 const zeroPrv = Vec3::Zero();
    EXPECT_EQ(subPrv(prv, zeroPrv), prv);
    EXPECT_EQ(subPrv(zeroPrv, prv), Vec3(-prv));
}

// The relative rotation between two equal PRVs is the zero PRV, up to rounding.
TYPED_TEST(RepresentationTransformTest, subPrvOfSameIsZero) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;

    Vec3 const prv(T(0.1), T(-0.2), T(0.3));
    EXPECT_TRUE(subPrv(prv, prv).isZero(T(10) * std::numeric_limits<T>::epsilon()));
}

// For rotations about the same axis, the relative angle is the difference of the angles: 150 - (-150) = 300 deg.
// The result is not switched to the short rotation (-60 deg).
TYPED_TEST(RepresentationTransformTest, subPrvSameAxis) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;

    T const degToRad = std::numbers::pi_v<T> / T(180);
    Vec3 const axis(T(1), T(0), T(0));
    Vec3 const prv1 = T(150) * degToRad * axis;
    Vec3 const prv2 = T(-150) * degToRad * axis;
    Vec3 const expected = T(300) * degToRad * axis;

    EXPECT_TRUE(subPrv(prv1, prv2).isApprox(expected, T(10) * std::numeric_limits<T>::epsilon()));
}

// The result is the PRV of frame 1 relative to frame 2, so its DCM is [C1] [C2]^T. The DCMs are built
// independently with Eigen::AngleAxis (the DCM is the transpose of its matrix).
TYPED_TEST(RepresentationTransformTest, subPrvFrameOrder) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;
    using Mat3 = Eigen::Matrix3<T>;

    T const degToRad = std::numbers::pi_v<T> / T(180);
    Vec3 const axis1(T(1), T(0), T(0));
    Vec3 const axis2(T(0), T(0.6), T(0.8));
    T const phi1 = T(70) * degToRad;
    T const phi2 = T(50) * degToRad;

    Vec3 const prv1 = phi1 * axis1;
    Vec3 const prv2 = phi2 * axis2;
    Mat3 const dcm1 = Eigen::AngleAxis<T>(phi1, axis1).toRotationMatrix().transpose();
    Mat3 const dcm2 = Eigen::AngleAxis<T>(phi2, axis2).toRotationMatrix().transpose();

    EXPECT_TRUE(
        prvToDcm(subPrv(prv1, prv2)).isApprox(dcm1 * dcm2.transpose(), T(10) * std::numeric_limits<T>::epsilon()));
}

// Small rotations must not be lost. Double only: in float these rotations are too small to see.
//  - A 1e-9 rad first rotation still changes the result. A shortcut that returns prv1 - prv2 would be off by
//    1/2 (prv1 x prv2), about 5e-10 rad here.
//  - Two rotations 1e-8 rad apart leave 1e-8 rad. A formula using 2*acos(cos(angle/2)) loses it.
TEST(RigidBodyKinematics, subPrvSmallRotations) {
    using Mat3 = Eigen::Matrix3d;

    Eigen::Vector3d const xAxis(1.0, 0.0, 0.0);
    Eigen::Vector3d const yAxis(0.0, 1.0, 0.0);
    double const tol = 10.0 * std::numeric_limits<double>::epsilon();

    double const tinyAngle = 1e-9;
    Mat3 const tinyDcm = Eigen::AngleAxisd(tinyAngle, xAxis).toRotationMatrix().transpose();
    Mat3 const yDcm = Eigen::AngleAxisd(1.0, yAxis).toRotationMatrix().transpose();
    EXPECT_TRUE(prvToDcm(subPrv(Eigen::Vector3d(tinyAngle * xAxis), yAxis)).isApprox(tinyDcm * yDcm.transpose(), tol));

    double const netAngle = 1e-8;
    Mat3 const netDcm = Eigen::AngleAxisd(netAngle, xAxis).toRotationMatrix().transpose();
    EXPECT_TRUE(prvToDcm(subPrv(xAxis, Eigen::Vector3d((1.0 - netAngle) * xAxis))).isApprox(netDcm, tol));
}

TYPED_TEST(RepresentationTransformTest, epToEulerAngles321) {
    using T = TypeParam;
    using Vec4 = Eigen::Matrix<T, 4, 1>;
    using Vec3 = Eigen::Matrix<T, 3, 1>;

    T phi = TestFixture::dist(TestFixture::generator);
    Vec3 axis(TestFixture::dist(TestFixture::generator),
              TestFixture::dist(TestFixture::generator),
              TestFixture::dist(TestFixture::generator));
    axis.normalize();
    Vec4 ep(std::cos(phi / 2), std::sin(phi / 2) * axis(0), std::sin(phi / 2) * axis(1), std::sin(phi / 2) * axis(2));

    T expectedArray[3] = {};
    if constexpr (std::is_same_v<T, float>) {
        EP2Euler321_float(ep.data(), expectedArray);
    } else {
        EP2Euler321(ep.data(), expectedArray);
    }

    Vec3 expected = Eigen::Map<Vec3>(expectedArray);
    Vec3 result = epToEulerAngles321<T>(ep);

    T const tolerance = std::max(rotationParameterTolerance(expected), TestFixture::accuracy);
    EXPECT_LT(relativeErrorNorm(result, expected), tolerance);
}

TYPED_TEST(RepresentationTransformTest, mrpToPrv) {
    using T = TypeParam;
    using Vec3 = typename RepresentationTransformTest<T>::Vec3;
    Vec3 rep(RepresentationTransformTest<T>::dist(RepresentationTransformTest<T>::generator),
             RepresentationTransformTest<T>::dist(RepresentationTransformTest<T>::generator),
             RepresentationTransformTest<T>::dist(RepresentationTransformTest<T>::generator));
    T expectedArray[3] = {};
    if constexpr (std::is_same_v<T, float>) {
        MRP2PRV_float(rep.data(), expectedArray);
    } else {
        MRP2PRV(rep.data(), expectedArray);
    }
    Vec3 expected = Eigen::Map<Vec3>(expectedArray);
    T const tolerance = std::max(rotationParameterTolerance(expected), RepresentationTransformTest<T>::accuracy);
    EXPECT_LT(relativeErrorNorm(mrpToPrv<T>(rep), expected), tolerance);
}

TYPED_TEST(RepresentationTransformTest, prvToMrp) {
    using T = TypeParam;
    using Vec3 = typename RepresentationTransformTest<T>::Vec3;
    Vec3 rep(RepresentationTransformTest<T>::dist(RepresentationTransformTest<T>::generator),
             RepresentationTransformTest<T>::dist(RepresentationTransformTest<T>::generator),
             RepresentationTransformTest<T>::dist(RepresentationTransformTest<T>::generator));
    T expectedArray[3] = {};
    if constexpr (std::is_same_v<T, float>) {
        PRV2MRP_float(rep.data(), expectedArray);
    } else {
        PRV2MRP(rep.data(), expectedArray);
    }
    Vec3 expected = Eigen::Map<Vec3>(expectedArray);
    T const tolerance = std::max(rotationParameterTolerance(expected), RepresentationTransformTest<T>::accuracy);
    EXPECT_LT(relativeErrorNorm(prvToMrp<T>(rep), expected), tolerance);
}

TYPED_TEST(RepresentationTransformTest, eulerAngles321ToMrp) {
    using T = TypeParam;
    using Vec3 = typename RepresentationTransformTest<T>::Vec3;
    Vec3 rep(RepresentationTransformTest<T>::dist(RepresentationTransformTest<T>::generator),
             RepresentationTransformTest<T>::dist(RepresentationTransformTest<T>::generator),
             RepresentationTransformTest<T>::dist(RepresentationTransformTest<T>::generator));
    T expectedArray[3] = {};
    if constexpr (std::is_same_v<T, float>) {
        Euler3212MRP_float(rep.data(), expectedArray);
    } else {
        Euler3212MRP(rep.data(), expectedArray);
    }
    Vec3 expected = Eigen::Map<Vec3>(expectedArray);
    T const tolerance = std::max(rotationParameterTolerance(expected), RepresentationTransformTest<T>::accuracy);
    EXPECT_LT(relativeErrorNorm(eulerAngles321ToMrp<T>(rep), expected), tolerance);
}

TYPED_TEST(RepresentationTransformTest, mrpToEulerAngles321) {
    using T = TypeParam;
    using Vec3 = typename RepresentationTransformTest<T>::Vec3;
    Vec3 rep(RepresentationTransformTest<T>::dist(RepresentationTransformTest<T>::generator),
             RepresentationTransformTest<T>::dist(RepresentationTransformTest<T>::generator),
             RepresentationTransformTest<T>::dist(RepresentationTransformTest<T>::generator));
    T expectedArray[3] = {};
    if constexpr (std::is_same_v<T, float>) {
        MRP2Euler321_float(rep.data(), expectedArray);
    } else {
        MRP2Euler321(rep.data(), expectedArray);
    }
    Vec3 expected = Eigen::Map<Vec3>(expectedArray);
    T const tolerance = std::max(rotationParameterTolerance(expected), RepresentationTransformTest<T>::accuracy);
    EXPECT_LT(relativeErrorNorm(mrpToEulerAngles321<T>(rep), expected), tolerance);
}

TYPED_TEST(RepresentationTransformTest, prvToEulerAngles321) {
    using T = TypeParam;
    using Vec3 = typename RepresentationTransformTest<T>::Vec3;
    Vec3 rep(RepresentationTransformTest<T>::dist(RepresentationTransformTest<T>::generator),
             RepresentationTransformTest<T>::dist(RepresentationTransformTest<T>::generator),
             RepresentationTransformTest<T>::dist(RepresentationTransformTest<T>::generator));
    T expectedArray[3] = {};
    if constexpr (std::is_same_v<T, float>) {
        PRV2Euler321_float(rep.data(), expectedArray);
    } else {
        PRV2Euler321(rep.data(), expectedArray);
    }
    Vec3 expected = Eigen::Map<Vec3>(expectedArray);
    T const tolerance = std::max(rotationParameterTolerance(expected), RepresentationTransformTest<T>::accuracy);
    EXPECT_LT(relativeErrorNorm(prvToEulerAngles321<T>(rep), expected), tolerance);
}

TYPED_TEST(RepresentationTransformTest, eulerAngles321ToPrv) {
    using T = TypeParam;
    using Vec3 = typename RepresentationTransformTest<T>::Vec3;
    Vec3 rep(RepresentationTransformTest<T>::dist(RepresentationTransformTest<T>::generator),
             RepresentationTransformTest<T>::dist(RepresentationTransformTest<T>::generator),
             RepresentationTransformTest<T>::dist(RepresentationTransformTest<T>::generator));
    T expectedArray[3] = {};
    if constexpr (std::is_same_v<T, float>) {
        Euler3212PRV_float(rep.data(), expectedArray);
    } else {
        Euler3212PRV(rep.data(), expectedArray);
    }
    Vec3 expected = Eigen::Map<Vec3>(expectedArray);
    T const tolerance = std::max(rotationParameterTolerance(expected), RepresentationTransformTest<T>::accuracy);
    EXPECT_LT(relativeErrorNorm(eulerAngles321ToPrv<T>(rep), expected), tolerance);
}

// Representation Derivatives Tests
template <typename T>
class RepresentationDerivativesTest : public ::testing::Test {
   public:
    using Vec3 = Eigen::Matrix<T, 3, 1>;

    std::default_random_engine generator{std::random_device{}()};
    std::uniform_real_distribution<T> dist{-std::numbers::pi, std::numbers::pi};
    T const accuracy = kinematicsAccuracy<T>();

    Vec3 randVec3() { return Vec3(dist(generator), dist(generator), dist(generator)); }
};

TYPED_TEST_SUITE(RepresentationDerivativesTest, FloatingPointTypes);

// ---------------------------------------------------------------------------
// dmrp: computes the MRP time derivative from the angular velocity, d(sigma)/dt = 1/4 [B(sigma)] omega.
// ---------------------------------------------------------------------------
TYPED_TEST(RepresentationDerivativesTest, dmrpMatchesCReference) {
    using T = TypeParam;
    typename TestFixture::Vec3 rep = this->randVec3();
    typename TestFixture::Vec3 rate = this->randVec3() / 10.0;

    T expectedArray[3] = {};
    if constexpr (std::is_same_v<T, float>) {
        dMRP_float(rep.data(), rate.data(), expectedArray);
    } else {
        dMRP(rep.data(), rate.data(), expectedArray);
    }

    typename TestFixture::Vec3 expected = Eigen::Map<typename TestFixture::Vec3>(expectedArray);
    T const tolerance = std::max(rotationParameterTolerance(expected), this->accuracy);
    EXPECT_LT(relativeErrorNorm(dmrp<T>(rep, rate), expected), tolerance);
}

// A zero angular velocity gives a zero MRP rate.
TYPED_TEST(RepresentationDerivativesTest, dmrpZeroRateIsZero) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;

    Vec3 const mrp(T(0.1), T(-0.2), T(0.3));
    Vec3 const zeroOmega = Vec3::Zero();
    EXPECT_EQ(dmrp(mrp, zeroOmega), Vec3::Zero());
}

// At the zero MRP set, [B] is the identity, so the MRP rate is omega / 4.
TYPED_TEST(RepresentationDerivativesTest, dmrpAtZeroMrp) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;

    Vec3 const zeroMrp = Vec3::Zero();
    Vec3 const omega(T(0.4), T(-0.8), T(1.2));
    EXPECT_EQ(dmrp(zeroMrp, omega), omega / T(4));
}

// For a rotation about a fixed axis, sigma = tan(phi / 4) * axis, so
// d(sigma)/dt = (1 / 4) * sec^2(phi / 4) * (d(phi)/dt) * axis.
TYPED_TEST(RepresentationDerivativesTest, dmrpRotationAboutFixedAxis) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;

    Vec3 const axis(T(0.48), T(0.36), T(-0.8));
    T const phi = T(90) * std::numbers::pi_v<T> / T(180);
    T const phiRate = T(0.3);

    Vec3 const mrp = std::tan(phi / T(4)) * axis;
    Vec3 const omega = phiRate * axis;

    // Calculate the expected MRP rate by differentiating sigma = tan(phi / 4) * axis.
    T const cosQuarterPhi = std::cos(phi / T(4));
    Vec3 const expected = phiRate / (T(4) * cosQuarterPhi * cosQuarterPhi) * axis;

    EXPECT_TRUE(dmrp(mrp, omega).isApprox(expected, T(10) * std::numeric_limits<T>::epsilon()));
}

// Verify the MRP derivative when angular velocity is perpendicular to the MRP vector.
// The nonzero cross product checks the sign and direction of the coupling term.
TYPED_TEST(RepresentationDerivativesTest, dmrpRatePerpendicularToMrp) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;

    T const s = T(0.5);
    T const w = T(0.2);
    Vec3 const mrp(s, T(0), T(0));
    Vec3 const omega(T(0), w, T(0));

    // sigma x omega = (0, 0, s * w).
    Vec3 const expected = Vec3(T(0), (T(1) - s * s) * w, T(2) * s * w) / T(4);

    EXPECT_TRUE(dmrp(mrp, omega).isApprox(expected, T(10) * std::numeric_limits<T>::epsilon()));
}

TYPED_TEST(RepresentationDerivativesTest, dmrpToOmega) {
    using T = TypeParam;
    typename TestFixture::Vec3 rep = this->randVec3();
    typename TestFixture::Vec3 rate = this->randVec3() / 10.0;

    T expectedArray[3] = {};
    if constexpr (std::is_same_v<T, float>) {
        dMRP2Omega_float(rep.data(), rate.data(), expectedArray);
    } else {
        dMRP2Omega(rep.data(), rate.data(), expectedArray);
    }

    typename TestFixture::Vec3 expected = Eigen::Map<typename TestFixture::Vec3>(expectedArray);
    T const tolerance = std::max(rotationParameterTolerance(expected), this->accuracy);
    EXPECT_LT(relativeErrorNorm(dmrpToOmega<T>(rep, rate), expected), tolerance);
}

TYPED_TEST(RepresentationDerivativesTest, dprv) {
    using T = TypeParam;
    typename TestFixture::Vec3 rep = this->randVec3();
    typename TestFixture::Vec3 rate = this->randVec3() / 10.0;

    T expectedArray[3] = {};
    if constexpr (std::is_same_v<T, float>) {
        dPRV_float(rep.data(), rate.data(), expectedArray);
    } else {
        dPRV(rep.data(), rate.data(), expectedArray);
    }

    typename TestFixture::Vec3 expected = Eigen::Map<typename TestFixture::Vec3>(expectedArray);
    T const tolerance = std::max(rotationParameterTolerance(expected), this->accuracy);
    EXPECT_LT(relativeErrorNorm(dprv<T>(rep, rate), expected), tolerance);
}

TYPED_TEST(RepresentationDerivativesTest, deuler321) {
    using T = TypeParam;
    typename TestFixture::Vec3 rep = this->randVec3();
    typename TestFixture::Vec3 rate = this->randVec3() / 10.0;

    T expectedArray[3] = {};
    if constexpr (std::is_same_v<T, float>) {
        dEuler321_float(rep.data(), rate.data(), expectedArray);
    } else {
        dEuler321(rep.data(), rate.data(), expectedArray);
    }

    typename TestFixture::Vec3 expected = Eigen::Map<typename TestFixture::Vec3>(expectedArray);
    T const tolerance = std::max(rotationParameterTolerance(expected), this->accuracy);
    EXPECT_LT(relativeErrorNorm(deuler321<T>(rep, rate), expected), tolerance);
}

TYPED_TEST(RepresentationDerivativesTest, ddmrp) {
    using T = TypeParam;
    typename TestFixture::Vec3 mrp = TestFixture::randVec3();
    typename TestFixture::Vec3 dmrp = TestFixture::randVec3();
    typename TestFixture::Vec3 omega = TestFixture::randVec3() / 10.0;
    typename TestFixture::Vec3 domega = TestFixture::randVec3() / 10.0;

    T expectedArray[3] = {};
    if constexpr (std::is_same_v<T, float>) {
        ddMRP_float(mrp.data(), dmrp.data(), omega.data(), domega.data(), expectedArray);
    } else {
        ddMRP(mrp.data(), dmrp.data(), omega.data(), domega.data(), expectedArray);
    }

    typename TestFixture::Vec3 expected = Eigen::Map<typename TestFixture::Vec3>(expectedArray);
    T const tolerance = std::max(rotationParameterTolerance(expected), TestFixture::accuracy);
    EXPECT_LT(relativeErrorNorm(ddmrp<T>(mrp, dmrp, omega, domega), expected), tolerance);
}

TYPED_TEST(RepresentationDerivativesTest, ddmrpToOmega) {
    using T = TypeParam;
    typename TestFixture::Vec3 mrp = TestFixture::randVec3();
    typename TestFixture::Vec3 dmrp = TestFixture::randVec3() / 10.0;
    typename TestFixture::Vec3 ddmrp = TestFixture::randVec3() / 100.0;

    T expectedArray[3] = {};
    if constexpr (std::is_same_v<T, float>) {
        ddMRP2dOmega_float(mrp.data(), dmrp.data(), ddmrp.data(), expectedArray);
    } else {
        ddMRP2dOmega(mrp.data(), dmrp.data(), ddmrp.data(), expectedArray);
    }

    typename TestFixture::Vec3 expected = Eigen::Map<typename TestFixture::Vec3>(expectedArray);
    T const tolerance = std::max(rotationParameterTolerance(expected), TestFixture::accuracy);
    EXPECT_LT(relativeErrorNorm(ddmrpTodOmega<T>(mrp, dmrp, ddmrp), expected), tolerance);
}

// ---------------------------------------------------------------------------
// mrpSwitch: keeps an MRP within the norm limit, otherwise returns its shadow set.
// ---------------------------------------------------------------------------
TYPED_TEST(RepresentationDerivativesTest, mrpSwitchMatchesCReference) {
    using T = TypeParam;
    typename TestFixture::Vec3 mrp = TestFixture::randVec3();
    // The MRP norm limit must be at least 1 (documented contract), so draw it from [1, 3].
    std::uniform_real_distribution<T> mrpNormLimitDistribution{T(1), T(3)};
    T const mrpNormLimit = mrpNormLimitDistribution(TestFixture::generator);

    T expectedArray[3] = {};
    if constexpr (std::is_same_v<T, float>) {
        MRPswitch_float(mrp.data(), mrpNormLimit, expectedArray);
    } else {
        MRPswitch(mrp.data(), mrpNormLimit, expectedArray);
    }

    typename TestFixture::Vec3 expected = Eigen::Map<typename TestFixture::Vec3>(expectedArray);
    T const tolerance = std::max(rotationParameterTolerance(expected), TestFixture::accuracy);
    EXPECT_LT(relativeErrorNorm(mrpSwitch<T>(mrp, mrpNormLimit), expected), tolerance);
}

// A zero MRP remains zero after switching.
TYPED_TEST(RepresentationDerivativesTest, mrpSwitchKeepsZero) {
    using T = TypeParam;
    Eigen::Vector3<T> const zeroMrp = Eigen::Vector3<T>::Zero();

    EXPECT_EQ(mrpSwitch(zeroMrp), zeroMrp);
}

// Keep MRPs within the norm limit (default 1) and switch those above it to their shadow set.
TYPED_TEST(RepresentationDerivativesTest, mrpSwitchBoundary) {
    using T = TypeParam;

    Eigen::Vector3<T> const mrpBelowNormLimit(T(0.9), 0, 0);
    Eigen::Vector3<T> const mrpAtNormLimit(T(1), 0, 0);
    Eigen::Vector3<T> const mrpAboveNormLimit(T(2), 0, 0);
    Eigen::Vector3<T> const shadowOfMrpAboveNormLimit(T(-0.5), 0, 0);

    EXPECT_EQ(mrpSwitch(mrpBelowNormLimit), mrpBelowNormLimit);
    EXPECT_EQ(mrpSwitch(mrpAtNormLimit), mrpAtNormLimit);
    EXPECT_EQ(mrpSwitch(mrpAboveNormLimit), shadowOfMrpAboveNormLimit);
}

// ---------------------------------------------------------------------------
// mrpShadow: maps an MRP set to its shadow set, which describes the same attitude.
// ---------------------------------------------------------------------------
TYPED_TEST(RepresentationDerivativesTest, mrpShadowMatchesCReference) {
    using T = TypeParam;
    typename TestFixture::Vec3 mrp = TestFixture::randVec3();

    T expectedArray[3] = {};
    if constexpr (std::is_same_v<T, float>) {
        MRPshadow_float(mrp.data(), expectedArray);
    } else {
        MRPshadow(mrp.data(), expectedArray);
    }

    typename TestFixture::Vec3 expected = Eigen::Map<typename TestFixture::Vec3>(expectedArray);
    T const tolerance = std::max(rotationParameterTolerance(expected), TestFixture::accuracy);
    EXPECT_LT(relativeErrorNorm(mrpShadow<T>(mrp), expected), tolerance);
}

// Verify the shadow transformation and its inverse.
TYPED_TEST(RepresentationDerivativesTest, mrpShadowKnownValues) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;
    using Mat3 = Eigen::Matrix<T, 3, 3>;

    Vec3 const mrp(T(1), T(1), T(0));
    Vec3 const shadow = mrpShadow(mrp);

    // Verify the expected shadow set.
    EXPECT_EQ(shadow, Vec3(T(-0.5), T(-0.5), T(0)));

    // Applying the shadow transformation twice restores the original MRP.
    EXPECT_EQ(mrpShadow(shadow), mrp);

    // The product of the squared MRP norms is 1.
    EXPECT_EQ(mrp.squaredNorm() * shadow.squaredNorm(), T(1));

    // The original and shadow MRP sets represent the same attitude.
    Mat3 const dcmMrp = mrpToDcm(mrp);
    Mat3 const dcmShadow = mrpToDcm(shadow);

    EXPECT_LT(relativeErrorNorm(dcmShadow, dcmMrp), rotationParameterTolerance(dcmMrp));
}

// The shadow set is undefined for a zero MRP.
TYPED_TEST(RepresentationDerivativesTest, mrpShadowOfZeroIsNonFinite) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;

    Vec3 const zeroMrp = Vec3::Zero();
    EXPECT_FALSE(mrpShadow(zeroMrp).allFinite());
}

TYPED_TEST(RepresentationDerivativesTest, rotationMatrix) {
    using T = TypeParam;
    T expectedArray[3][3] = {};

    for (int axis = 1; axis <= 3; ++axis) {
        T angle = TestFixture::dist(TestFixture::generator);
        if constexpr (std::is_same_v<T, float>) {
            Mi_float(angle, axis, expectedArray);
        } else {
            Mi(angle, axis, expectedArray);
        }

        Eigen::Matrix<T, 3, 3> expected = cArray33ToEigenMatrix33(expectedArray);
        T const tolerance = std::max(rotationParameterTolerance(expected), TestFixture::accuracy);
        EXPECT_LT(relativeErrorNorm(rotationMatrix<T>(angle, axis), expected), tolerance);
    }
}

// ---------------------------------------------------------------------------
// tildeMatrix: computes the skew-symmetric cross-product matrix, [v~] u = v x u.
// ---------------------------------------------------------------------------
TYPED_TEST(RepresentationDerivativesTest, tildeMatrixMatchesCReference) {
    using T = TypeParam;
    typename TestFixture::Vec3 vec = TestFixture::randVec3();
    typename TestFixture::Vec3 testVec = TestFixture::randVec3();

    T expectedArray[3][3] = {};
    if constexpr (std::is_same_v<T, float>) {
        tilde_float(vec.data(), expectedArray);
    } else {
        tilde(vec.data(), expectedArray);
    }

    Eigen::Matrix<T, 3, 3> expected = cArray33ToEigenMatrix33(expectedArray);
    auto const zeroVec = TestFixture::Vec3::Zero();
    T const zeroTolerance = std::max(rotationParameterTolerance(vec), TestFixture::accuracy);
    EXPECT_LT(relativeErrorNorm(tildeMatrix<T>(vec) * vec, zeroVec), zeroTolerance);

    T const crossTolerance =
        std::max({rotationParameterTolerance(vec), rotationParameterTolerance(testVec), TestFixture::accuracy});
    EXPECT_LT(relativeErrorNorm(tildeMatrix<T>(vec) * testVec, vec.cross(testVec)), crossTolerance);

    T const matrixTolerance = std::max(rotationParameterTolerance(expected), TestFixture::accuracy);
    EXPECT_LT(relativeErrorNorm(tildeMatrix<T>(vec), expected), matrixTolerance);
}

// The tilde matrix of the zero vector is the zero matrix.
TYPED_TEST(RepresentationDerivativesTest, tildeMatrixOfZeroIsZero) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;
    using Mat3 = Eigen::Matrix3<T>;

    Vec3 const zeroVector = Vec3::Zero();
    EXPECT_EQ(tildeMatrix(zeroVector), Mat3::Zero());
}

// Verify [v~] u = v x u for the unit axes: X x Y = Z, Y x Z = X and Z x X = Y.
TYPED_TEST(RepresentationDerivativesTest, tildeMatrixUnitAxes) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;

    Vec3 const unitX = Vec3::UnitX();
    Vec3 const unitY = Vec3::UnitY();
    Vec3 const unitZ = Vec3::UnitZ();

    EXPECT_EQ(tildeMatrix(unitX) * unitY, unitZ);
    EXPECT_EQ(tildeMatrix(unitY) * unitZ, unitX);
    EXPECT_EQ(tildeMatrix(unitZ) * unitX, unitY);
}

// The tilde matrix is skew-symmetric: [v~]^T = -[v~].
TYPED_TEST(RepresentationDerivativesTest, tildeMatrixIsSkewSymmetric) {
    using T = TypeParam;
    using Vec3 = Eigen::Vector3<T>;
    using Mat3 = Eigen::Matrix3<T>;

    Vec3 const vector(T(1), T(-2), T(3));
    Mat3 const tilde = tildeMatrix(vector);
    EXPECT_EQ(tilde.transpose(), Mat3(-tilde));
}

// ---------------------------------------------------------------------------
// Near-identity accuracy, compared against analytic truth.
// Near identity cos(phi/2) rounds to 1.0, so 2*acos(cos(phi/2)) would lose the angle, while
// 2*atan2(|vec|, scalar) keeps it. kCondTol is far tighter than the acos error (~1e-11) and loose
// compared with the atan2 error (~1e-16).
// ---------------------------------------------------------------------------
namespace {
constexpr double kCondTol = 1e-12;
Eigen::Vector3d const kCondAxis = Eigen::Vector3d(1.0, -2.0, 3.0).normalized();
}  // namespace

TEST(ConditioningNearIdentity, EpToPrvSmallAngle) {
    for (double phi : {1e-2, 1e-4, 1e-6, 1e-8}) {
        Eigen::Vector4d ep;
        ep(0) = std::cos(phi / 2);
        ep.tail<3>() = std::sin(phi / 2) * kCondAxis;
        Eigen::Vector3d const expected = phi * kCondAxis;
        EXPECT_LT(relativeErrorNorm(epToPrv<double>(ep), expected), kCondTol) << "phi=" << phi;
    }
}

// ---------------------------------------------------------------------------
// Domain robustness: degenerate inputs must stay finite (no acos-domain NaN, no 0/0).
// ---------------------------------------------------------------------------
TEST(DomainRobustness, EpToPrvSlightlyOffUnit) {
    // q0 just above 1 from integer-decode / renorm rounding: acos -> NaN, atan2 stays finite.
    Eigen::Vector4d const ep(1.0 + 1e-9, 1e-6, -2e-6, 5e-7);
    EXPECT_TRUE(epToPrv<double>(ep).allFinite());
}

// ---------------------------------------------------------------------------
// Euler-321 extraction approaching the +-90 deg pitch singularity. The atan2 pitch
// stays finite and accurate where asin(sin) is ill-conditioned / can NaN on a
// rounded-past-1 argument.
// ---------------------------------------------------------------------------
TEST(Euler321NearGimbalLock, ExtractionAccurateApproachingNinety) {
    double const yaw = 0.3;
    double const roll = -0.2;
    for (double pitchDeg : {89.0, 89.9, 89.99}) {
        double const pitch = pitchDeg * std::numbers::pi / 180.0;
        Eigen::Vector3d const eIn(yaw, pitch, roll);
        Eigen::Vector4d const ep = eulerAngles321ToEp<double>(eIn);
        Eigen::Vector3d const eFromEp = epToEulerAngles321<double>(ep);
        Eigen::Vector3d const eFromDcm = dcmToEulerAngles321<double>(epToDcm<double>(ep));
        EXPECT_TRUE(eFromEp.allFinite());
        EXPECT_TRUE(eFromDcm.allFinite());
        EXPECT_LT(relativeErrorNorm(eFromEp, eIn), 1e-9) << "epToEuler pitch=" << pitchDeg;
        EXPECT_LT(relativeErrorNorm(eFromDcm, eIn), 1e-9) << "dcmToEuler pitch=" << pitchDeg;
    }
}

TEST(Euler321NearGimbalLock, ExactNinetyStaysFinite) {
    // sin(pitch) can round just above 1: asin -> NaN, atan2 -> finite pi/2. At exact gimbal
    // lock only the pitch is uniquely recoverable (yaw/roll couple), so only pitch is checked.
    double const pitch = std::numbers::pi / 2.0;
    Eigen::Vector3d const eIn(0.3, pitch, -0.2);
    Eigen::Vector4d const ep = eulerAngles321ToEp<double>(eIn);
    Eigen::Vector3d const eFromEp = epToEulerAngles321<double>(ep);
    Eigen::Vector3d const eFromDcm = dcmToEulerAngles321<double>(epToDcm<double>(ep));
    EXPECT_TRUE(eFromEp.allFinite());
    EXPECT_TRUE(eFromDcm.allFinite());
    EXPECT_NEAR(eFromEp(1), pitch, 1e-9);
    EXPECT_NEAR(eFromDcm(1), pitch, 1e-9);
}

// ---------------------------------------------------------------------------
// C/C++ parity at near-identity and near-singular inputs. The C and C++ functions both use
// atan2, so they agree here; an acos or asin version would not.
// ---------------------------------------------------------------------------
TEST(CCppParityNearIdentity, EpToPrvSmallAngle) {
    double const phi = 1e-7;
    Eigen::Vector4d ep;
    ep(0) = std::cos(phi / 2);
    ep.tail<3>() = std::sin(phi / 2) * kCondAxis;
    double cRef[3] = {};
    EP2PRV(ep.data(), cRef);
    EXPECT_LT(relativeErrorNorm(epToPrv<double>(ep), Eigen::Map<Eigen::Vector3d>(cRef)), 1e-12);
}

TEST(CCppParityNearIdentity, SubPrvNearIdentity) {
    double const phi = 1.0;
    double const eps = 1e-7;
    Eigen::Vector3d a = phi * kCondAxis;
    Eigen::Vector3d b = (phi - eps) * kCondAxis;
    double cRef[3] = {};
    subPRV(a.data(), b.data(), cRef);
    EXPECT_LT(relativeErrorNorm(subPrv<double>(a, b), Eigen::Map<Eigen::Vector3d>(cRef)), 1e-12);
}

TEST(CCppParityNearIdentity, AddPrvNearCancellation) {
    double const phi = 1.0;
    double const eps = 1e-7;
    Eigen::Vector3d a = phi * kCondAxis;
    Eigen::Vector3d b = -(phi - eps) * kCondAxis;
    double cRef[3] = {};
    addPRV(a.data(), b.data(), cRef);
    EXPECT_LT(relativeErrorNorm(addPrv<double>(a, b), Eigen::Map<Eigen::Vector3d>(cRef)), 1e-12);
}

TEST(CCppParityNearIdentity, Euler321NearGimbal) {
    double const pitch = 89.99 * std::numbers::pi / 180.0;
    Eigen::Vector3d const eIn(0.3, pitch, -0.2);
    Eigen::Matrix3d const dcm = epToDcm<double>(eulerAngles321ToEp<double>(eIn));
    double cArr[3][3];
    eigenMatrix3ToCArray(dcm, cArr);
    double cRef[3] = {};
    C2Euler321(cArr, cRef);
    EXPECT_LT(relativeErrorNorm(dcmToEulerAngles321<double>(dcm), Eigen::Map<Eigen::Vector3d>(cRef)), 1e-10);
}
