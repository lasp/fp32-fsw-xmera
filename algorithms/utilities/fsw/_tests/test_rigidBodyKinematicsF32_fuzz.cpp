#include "utilities/fsw/rigidBodyKinematics.hpp"
#include "utilities/testUtilities/eigenFuzzDomains.hpp"

#include <fuzztest/fuzztest.h>
#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>
#include <limits>
#include <numbers>
#include <type_traits>

namespace {
//! Build a unit quaternion from four arbitrary doubles, falling back to identity near zero.
Eigen::Vector4d unitQuaternion(double q0, double q1, double q2, double q3) {
    Eigen::Vector4d ep(q0, q1, q2, q3);
    double const n = ep.norm();
    if (n < 1e-9) {
        return {1.0, 0.0, 0.0, 0.0};
    }
    return ep / n;
}

bool closeRel(Eigen::Vector3d const& a, Eigen::Vector3d const& b, double tol) {
    return (a - b).norm() <= tol * (1.0 + b.norm());
}

// prv -> ep -> prv round-trips for ||prv|| < pi.
void roundTripPrvEp(double px, double py, double pz) {
    Eigen::Vector3d const prv(px, py, pz);
    Eigen::Vector3d const out = epToPrv<double>(prvToEp<double>(prv));
    ASSERT_TRUE(out.allFinite());
    EXPECT_TRUE(closeRel(out, prv, 1e-9));
}

FUZZ_TEST(RigidBodyKinematicsFuzz, roundTripPrvEp)
    .WithDomains(fuzztest::InRange(-1.0, 1.0), fuzztest::InRange(-1.0, 1.0), fuzztest::InRange(-1.0, 1.0));

// mrp -> ep -> mrp round-trips for the canonical set (||mrp|| < 1).
void roundTripMrpEp(double mx, double my, double mz) {
    Eigen::Vector3d const mrp(mx, my, mz);
    Eigen::Vector3d const out = epToMrp<double>(mrpToEp<double>(mrp));
    ASSERT_TRUE(out.allFinite());
    EXPECT_TRUE(closeRel(out, mrp, 1e-9));
}

FUZZ_TEST(RigidBodyKinematicsFuzz, roundTripMrpEp)
    .WithDomains(fuzztest::InRange(-0.5, 0.5), fuzztest::InRange(-0.5, 0.5), fuzztest::InRange(-0.5, 0.5));

// Any unit EP maps to a proper orthonormal DCM.
void epToDcmProperOrthonormal(double q0, double q1, double q2, double q3) {
    Eigen::Matrix3d const r = epToDcm<double>(unitQuaternion(q0, q1, q2, q3));
    ASSERT_TRUE(r.allFinite());
    EXPECT_LT((r * r.transpose() - Eigen::Matrix3d::Identity()).norm(), 1e-12);
    EXPECT_NEAR(r.determinant(), 1.0, 1e-12);
}

FUZZ_TEST(RigidBodyKinematicsFuzz, epToDcmProperOrthonormal)
    .WithDomains(fuzztest::InRange(-1.0, 1.0),
                 fuzztest::InRange(-1.0, 1.0),
                 fuzztest::InRange(-1.0, 1.0),
                 fuzztest::InRange(-1.0, 1.0));

// ---------------------------------------------------------------------------
// addPrv: computes the PRV of two successive rotations, first prv1, then prv2.
// ---------------------------------------------------------------------------
template <typename T>
void addPrvProperties(const Eigen::Vector3<T>& direction1, T angle1, const Eigen::Vector3<T>& direction2, T angle2) {
    using Vec3 = Eigen::Vector3<T>;
    using Mat3 = Eigen::Matrix3<T>;

    // A zero direction has no rotation axis; the zero PRV is covered by the addPrvWithZero GTest.
    if (direction1.isZero(T(0)) || direction2.isZero(T(0))) {
        return;
    }
    Vec3 const axis1 = direction1.stableNormalized();
    Vec3 const axis2 = direction2.stableNormalized();

    Vec3 const resultPrv = addPrv(Vec3(angle1 * axis1), Vec3(angle2 * axis2));

    // The result must contain only finite values.
    ASSERT_TRUE(resultPrv.allFinite());

    // The DCM of the result must be [C2] [C1], built independently with Eigen::AngleAxis (the DCM is the transpose
    // of its matrix). The comparison has three DCMs (the result and the two inputs), each within 10 eps as in the
    // prvToDcm tests, so the two sides can differ by up to 30 eps.
    Mat3 const dcm1 = Eigen::AngleAxis<T>(angle1, axis1).toRotationMatrix().transpose();
    Mat3 const dcm2 = Eigen::AngleAxis<T>(angle2, axis2).toRotationMatrix().transpose();
    T const tol = T(30) * std::numeric_limits<T>::epsilon();
    EXPECT_TRUE(prvToDcm(resultPrv).isApprox(dcm2 * dcm1, tol));
}

void addPrvPropertiesFloat(const Eigen::Vector3f& direction1,
                           float angle1,
                           const Eigen::Vector3f& direction2,
                           float angle2) {
    addPrvProperties(direction1, angle1, direction2, angle2);
}

void addPrvPropertiesDouble(const Eigen::Vector3d& direction1,
                            double angle1,
                            const Eigen::Vector3d& direction2,
                            double angle2) {
    addPrvProperties(direction1, angle1, direction2, angle2);
}

// Each input rotation is 0 to 180 deg, so the result is 0 to 360 deg, including long rotations above 180 deg.
FUZZ_TEST(RigidBodyKinematicsFuzz, addPrvPropertiesFloat)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1.0F, 1.0F),            // Random PRV 1 axis components.
                 fuzztest::InRange(0.0F, std::numbers::pi_v<float>),   // PRV 1 angles from 0 to 180 deg.
                 xmera::fuzz::Vector3fInRange(-1.0F, 1.0F),            // Random PRV 2 axis components.
                 fuzztest::InRange(0.0F, std::numbers::pi_v<float>));  // PRV 2 angles from 0 to 180 deg.

FUZZ_TEST(RigidBodyKinematicsFuzz, addPrvPropertiesDouble)
    .WithDomains(xmera::fuzz::Vector3dInRange(-1.0, 1.0),    // Random PRV 1 axis components.
                 fuzztest::InRange(0.0, std::numbers::pi),   // PRV 1 angles from 0 to 180 deg.
                 xmera::fuzz::Vector3dInRange(-1.0, 1.0),    // Random PRV 2 axis components.
                 fuzztest::InRange(0.0, std::numbers::pi));  // PRV 2 angles from 0 to 180 deg.

// subPrv undoes addPrv: subPrv(addPrv(b, a), b) == a (small domains keep the composition < pi).
// PRV composition is non-commutative: addPrv(b, a) = qb (x) qa and subPrv(., b) removes qb from
// the left, so b must be applied first for the round-trip to recover a (not a conjugation of it).
void addSubPrvInverse(double ax, double ay, double az, double bx, double by, double bz) {
    Eigen::Vector3d const a(ax, ay, az);
    Eigen::Vector3d const b(bx, by, bz);
    Eigen::Vector3d const out = subPrv<double>(addPrv<double>(b, a), b);
    ASSERT_TRUE(out.allFinite());
    EXPECT_TRUE(closeRel(out, a, 1e-8));
}

FUZZ_TEST(RigidBodyKinematicsFuzz, addSubPrvInverse)
    .WithDomains(fuzztest::InRange(-0.5, 0.5),
                 fuzztest::InRange(-0.5, 0.5),
                 fuzztest::InRange(-0.5, 0.5),
                 fuzztest::InRange(-0.5, 0.5),
                 fuzztest::InRange(-0.5, 0.5),
                 fuzztest::InRange(-0.5, 0.5));

// Euler-321 extraction is always finite, including near gimbal lock.
void euler321ExtractionFinite(double q0, double q1, double q2, double q3) {
    Eigen::Vector4d const ep = unitQuaternion(q0, q1, q2, q3);
    EXPECT_TRUE(dcmToEulerAngles321<double>(epToDcm<double>(ep)).allFinite());
    EXPECT_TRUE(epToEulerAngles321<double>(ep).allFinite());
}

FUZZ_TEST(RigidBodyKinematicsFuzz, euler321ExtractionFinite)
    .WithDomains(fuzztest::InRange(-1.0, 1.0),
                 fuzztest::InRange(-1.0, 1.0),
                 fuzztest::InRange(-1.0, 1.0),
                 fuzztest::InRange(-1.0, 1.0));

// ---------------------------------------------------------------------------
// mrpShadow: maps an MRP set to its shadow set, which describes the same attitude.
// ---------------------------------------------------------------------------
template <typename T>
void mrpShadowProperties(const Eigen::Vector3<T>& direction, T log10MrpNorm) {
    using Vec3 = Eigen::Vector3<T>;

    // The zero MRP has no shadow set.
    if (direction.isZero(T(0))) {
        return;
    }

    // Generate direction and magnitude independently to exercise vectors across
    // multiple orders of magnitude while retaining coverage of arbitrary directions.
    // Directly sampling components over a wide range would rarely reach small
    // magnitudes or specific norm boundaries.
    Vec3 const mrp = direction.stableNormalized() * std::pow(T(10), log10MrpNorm);
    Vec3 const shadow = mrpShadow(mrp);
    T const tol = T(10) * std::numeric_limits<T>::epsilon();

    // Verify that the shadow MRP contains only finite values.
    ASSERT_TRUE(shadow.allFinite());

    // The shadow MRP points in the opposite direction.
    EXPECT_TRUE(shadow.stableNormalized().isApprox(-mrp.stableNormalized(), tol));

    // The product of the squared MRP norms is 1.
    EXPECT_NEAR(mrp.squaredNorm() * shadow.squaredNorm(), T(1), tol);

    // Applying the shadow transformation twice restores the original MRP.
    EXPECT_TRUE(mrpShadow(shadow).isApprox(mrp, tol));

    // The original and shadow MRP sets represent the same attitude.
    // Limit DCM comparisons to magnitudes with finite intermediate calculations: (1 + |mrp|^2)^2 in mrpToDcm
    // overflows above |mrp| ~4e9 in float and ~1e77 in double.
    T const maxLog10MrpNormForDcm = std::is_same_v<T, float> ? T(9) : T(76);

    if (std::abs(log10MrpNorm) <= maxLog10MrpNormForDcm) {
        EXPECT_TRUE(mrpToDcm(shadow).isApprox(mrpToDcm(mrp), tol));
    }
}

void mrpShadowPropertiesFloat(const Eigen::Vector3f& direction, float log10MrpNorm) {
    mrpShadowProperties(direction, log10MrpNorm);
}

void mrpShadowPropertiesDouble(const Eigen::Vector3d& direction, double log10MrpNorm) {
    mrpShadowProperties(direction, log10MrpNorm);
}

FUZZ_TEST(RigidBodyKinematicsFuzz, mrpShadowPropertiesFloat)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1.0F, 1.0F),  // Random X, Y, Z direction components.
                 fuzztest::InRange(-18.0F, 18.0F));          // MRP norms from 1e-18 to 1e18.

FUZZ_TEST(RigidBodyKinematicsFuzz, mrpShadowPropertiesDouble)
    .WithDomains(xmera::fuzz::Vector3dInRange(-1.0, 1.0),  // Random X, Y, Z direction components.
                 fuzztest::InRange(-150.0, 150.0));        // MRP norms from 1e-150 to 1e150.

// ---------------------------------------------------------------------------
// mrpSwitch: keeps an MRP within the norm limit, otherwise returns its shadow set.
// ---------------------------------------------------------------------------
template <typename T>
void mrpSwitchProperties(const Eigen::Vector3<T>& mrp, T mrpNormLimit) {
    Eigen::Vector3<T> const result = mrpSwitch(mrp, mrpNormLimit);

    // The result never exceeds the norm limit.
    EXPECT_LE(result.norm(), mrpNormLimit);

    // An MRP set within the limit is returned unchanged. The exact boundary is covered by the mrpSwitchBoundary GTest.
    if (mrp.norm() < mrpNormLimit) {
        EXPECT_EQ(result, mrp);
    }

    // The result describes the same attitude as the input: an MRP set and its shadow set give the same DCM.
    EXPECT_TRUE(mrpToDcm(result).isApprox(mrpToDcm(mrp), T(10) * std::numeric_limits<T>::epsilon()));

    // With the default limit of 1, mrpSwitch(mrp) is mrpSwitch(mrp, 1).
    EXPECT_EQ(mrpSwitch(mrp), mrpSwitch(mrp, T(1)));
}

void mrpSwitchPropertiesFloat(const Eigen::Vector3f& mrp, float mrpNormLimit) {
    mrpSwitchProperties(mrp, mrpNormLimit);
}

void mrpSwitchPropertiesDouble(const Eigen::Vector3d& mrp, double mrpNormLimit) {
    mrpSwitchProperties(mrp, mrpNormLimit);
}

// Fuzz MRP components and norm limits on both sides of the switching condition.
FUZZ_TEST(RigidBodyKinematicsFuzz, mrpSwitchPropertiesFloat)
    .WithDomains(xmera::fuzz::Vector3fInRange(-2.0F, 2.0F),  // Random MRP components.
                 fuzztest::InRange(1.0F, 3.0F));             // MRP norm limits from 1 to 3.

FUZZ_TEST(RigidBodyKinematicsFuzz, mrpSwitchPropertiesDouble)
    .WithDomains(xmera::fuzz::Vector3dInRange(-2.0, 2.0),  // Random MRP components.
                 fuzztest::InRange(1.0, 3.0));             // MRP norm limits from 1 to 3.

// ---------------------------------------------------------------------------
// dcmToMrp: converts a DCM to the principal MRP set with norm at most 1.
// ---------------------------------------------------------------------------
template <typename T>
void dcmToMrpProperties(const Eigen::Vector3<T>& direction, T log10MrpNorm) {
    using Vec3 = Eigen::Vector3<T>;
    using Mat3 = Eigen::Matrix<T, 3, 3>;

    // A zero direction produces a zero MRP, representing the identity rotation.
    Vec3 inputMrp = Vec3::Zero();
    if (!direction.isZero(T(0))) {
        inputMrp = direction.stableNormalized() * std::pow(T(10), log10MrpNorm);
    }

    Mat3 const dcm = mrpToDcm(inputMrp);
    Vec3 const resultMrp = dcmToMrp(dcm);
    T const tol = T(10) * std::numeric_limits<T>::epsilon();

    // Verify that the converted MRP contains only finite values.
    ASSERT_TRUE(resultMrp.allFinite());

    // The principal MRP norm must not exceed 1, allowing for rounding at 180 degrees.
    EXPECT_LE(resultMrp.norm(), T(1) + tol);

    // Converting the result back to a DCM must preserve the original attitude.
    EXPECT_TRUE(mrpToDcm(resultMrp).isApprox(dcm, tol));

    T const inputMrpNorm = inputMrp.stableNorm();
    if (inputMrpNorm < T(1) - tol) {
        // Principal MRPs must be recovered unchanged.
        EXPECT_TRUE(resultMrp.isApprox(inputMrp, tol));
    } else if (inputMrpNorm > T(1) + tol) {
        // Non-principal MRPs must be converted to their shadow set.
        EXPECT_TRUE(resultMrp.isApprox(mrpShadow(inputMrp), tol));
    } else {
        // Near the 180-degree boundary, rounding can affect principal/shadow
        // selection. Accept either equivalent MRP representation within tolerance.
        EXPECT_TRUE(resultMrp.isApprox(inputMrp, tol) || resultMrp.isApprox(mrpShadow(inputMrp), tol));
    }
}

void dcmToMrpPropertiesFloat(const Eigen::Vector3f& direction, float log10MrpNorm) {
    dcmToMrpProperties(direction, log10MrpNorm);
}

void dcmToMrpPropertiesDouble(const Eigen::Vector3d& direction, double log10MrpNorm) {
    dcmToMrpProperties(direction, log10MrpNorm);
}

// The largest norms stay below the mrpToDcm overflow (|mrp| ~4e9 in float, ~1e77 in double).
// The second range (MRP norms 0.9977 to 1.0023) tests rotations close to 180 deg, where the MRP norm is 1.
FUZZ_TEST(RigidBodyKinematicsFuzz, dcmToMrpPropertiesFloat)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1.0F, 1.0F),             // Random X, Y, Z direction components.
                 fuzztest::OneOf(fuzztest::InRange(-18.0F, 9.0F),       // MRP norms from 1e-18 to 1e9.
                                 fuzztest::InRange(-0.001F, 0.001F)));  // Rotations from 179.74 to 180.26 deg.

FUZZ_TEST(RigidBodyKinematicsFuzz, dcmToMrpPropertiesDouble)
    .WithDomains(xmera::fuzz::Vector3dInRange(-1.0, 1.0),             // Random X, Y, Z direction components.
                 fuzztest::OneOf(fuzztest::InRange(-150.0, 76.0),     // MRP norms from 1e-150 to 1e76.
                                 fuzztest::InRange(-0.001, 0.001)));  // Rotations from 179.74 to 180.26 deg.

// ---------------------------------------------------------------------------
// mrpToDcm: converts an MRP set to its direction cosine matrix.
// ---------------------------------------------------------------------------
template <typename T>
void mrpToDcmProperties(const Eigen::Vector3<T>& direction, T log10MrpNorm) {
    using Vec3 = Eigen::Vector3<T>;
    using Mat3 = Eigen::Matrix<T, 3, 3>;

    // The zero MRP has no rotation axis; it is covered by the identity GTest.
    if (direction.isZero(T(0))) {
        return;
    }

    Vec3 const axis = direction.stableNormalized();
    Vec3 const mrp = axis * std::pow(T(10), log10MrpNorm);
    Mat3 const dcm = mrpToDcm(mrp);
    T const tol = T(10) * std::numeric_limits<T>::epsilon();

    // The DCM must contain only finite values.
    ASSERT_TRUE(dcm.allFinite());

    // The DCM must match the rotation by 4 atan(|mrp|) about the MRP axis, built independently with Eigen::AngleAxis.
    // AngleAxis gives the active rotation matrix; the DCM is its transpose. A match also makes the DCM orthonormal
    // with determinant +1.
    T const angle = T(4) * std::atan(mrp.stableNorm());
    Mat3 const expectedDcm = Eigen::AngleAxis<T>(angle, axis).toRotationMatrix().transpose();
    EXPECT_TRUE(dcm.isApprox(expectedDcm, tol));
}

void mrpToDcmPropertiesFloat(const Eigen::Vector3f& direction, float log10MrpNorm) {
    mrpToDcmProperties(direction, log10MrpNorm);
}

void mrpToDcmPropertiesDouble(const Eigen::Vector3d& direction, double log10MrpNorm) {
    mrpToDcmProperties(direction, log10MrpNorm);
}

// The largest norms stay below the mrpToDcm overflow (|mrp| ~4e9 in float, ~1e77 in double).
// The second range (MRP norms 0.9977 to 1.0023) tests rotations close to 180 deg, where the MRP norm is 1.
FUZZ_TEST(RigidBodyKinematicsFuzz, mrpToDcmPropertiesFloat)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1.0F, 1.0F),             // Random X, Y, Z direction components.
                 fuzztest::OneOf(fuzztest::InRange(-18.0F, 9.0F),       // MRP norms from 1e-18 to 1e9.
                                 fuzztest::InRange(-0.001F, 0.001F)));  // Rotations from 179.74 to 180.26 deg.

FUZZ_TEST(RigidBodyKinematicsFuzz, mrpToDcmPropertiesDouble)
    .WithDomains(xmera::fuzz::Vector3dInRange(-1.0, 1.0),             // Random X, Y, Z direction components.
                 fuzztest::OneOf(fuzztest::InRange(-150.0, 76.0),     // MRP norms from 1e-150 to 1e76.
                                 fuzztest::InRange(-0.001, 0.001)));  // Rotations from 179.74 to 180.26 deg.

// ---------------------------------------------------------------------------
// dcmToEp: converts a DCM to unit Euler parameters (scalar first) with q0 >= 0.
// ---------------------------------------------------------------------------
template <typename T>
void dcmToEpProperties(const Eigen::Vector3<T>& direction, T log10MrpNorm) {
    using Vec3 = Eigen::Vector3<T>;
    using Vec4 = Eigen::Vector4<T>;
    using Mat3 = Eigen::Matrix<T, 3, 3>;

    // A zero direction produces a zero MRP, representing the identity rotation.
    Vec3 inputMrp = Vec3::Zero();
    if (!direction.isZero(T(0))) {
        inputMrp = direction.stableNormalized() * std::pow(T(10), log10MrpNorm);
    }

    Mat3 const dcm = mrpToDcm(inputMrp);
    Vec4 const ep = dcmToEp(dcm);
    T const tol = T(10) * std::numeric_limits<T>::epsilon();

    // Verify that the Euler parameters contain only finite values.
    ASSERT_TRUE(ep.allFinite());

    // The scalar part q0 is never negative.
    EXPECT_GE(ep(0), T(0));

    // The Euler parameters have unit norm.
    EXPECT_NEAR(ep.norm(), T(1), tol);

    // Vector part of the Euler parameters of the input MRP: q_vec = 2 mrp / (1 + |mrp|^2). With the unit norm and
    // q0 >= 0, the vector part fixes the whole set. It is compared relative to its own size, so small rotations count.
    Vec3 const expectedVector = T(2) * inputMrp / (T(1) + inputMrp.squaredNorm());
    Vec3 const resultVector = ep.template tail<3>();

    T const inputMrpNorm = inputMrp.stableNorm();
    if (inputMrpNorm < T(1) - tol) {
        // Below norm 1 (rotations under 180 degrees), q0 > 0 and the vector part is unchanged.
        EXPECT_TRUE(resultVector.isApprox(expectedVector, tol));
    } else if (inputMrpNorm > T(1) + tol) {
        // Above norm 1, q0 would be negative, so the whole set is negated to keep q0 >= 0.
        EXPECT_TRUE(resultVector.isApprox(-expectedVector, tol));
    } else {
        // Near the 180-degree boundary q0 is about 0, so rounding can decide the sign.
        // Accept either equivalent set within tolerance.
        EXPECT_TRUE(resultVector.isApprox(expectedVector, tol) || resultVector.isApprox(-expectedVector, tol));
    }
}

void dcmToEpPropertiesFloat(const Eigen::Vector3f& direction, float log10MrpNorm) {
    dcmToEpProperties(direction, log10MrpNorm);
}

void dcmToEpPropertiesDouble(const Eigen::Vector3d& direction, double log10MrpNorm) {
    dcmToEpProperties(direction, log10MrpNorm);
}

// The DCMs are built from MRPs. The largest norms stay below the mrpToDcm overflow (|mrp| ~4e9 in float, ~1e77 in
// double). The second range (MRP norms 0.9977 to 1.0023) tests rotations close to 180 deg, where the q1, q2 and q3
// branches of Stanley's method are used.
FUZZ_TEST(RigidBodyKinematicsFuzz, dcmToEpPropertiesFloat)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1.0F, 1.0F),             // Random X, Y, Z direction components.
                 fuzztest::OneOf(fuzztest::InRange(-18.0F, 9.0F),       // MRP norms from 1e-18 to 1e9.
                                 fuzztest::InRange(-0.001F, 0.001F)));  // Rotations from 179.74 to 180.26 deg.

FUZZ_TEST(RigidBodyKinematicsFuzz, dcmToEpPropertiesDouble)
    .WithDomains(xmera::fuzz::Vector3dInRange(-1.0, 1.0),             // Random X, Y, Z direction components.
                 fuzztest::OneOf(fuzztest::InRange(-150.0, 76.0),     // MRP norms from 1e-150 to 1e76.
                                 fuzztest::InRange(-0.001, 0.001)));  // Rotations from 179.74 to 180.26 deg.

// ---------------------------------------------------------------------------
// epToMrp: converts Euler parameters to the principal MRP set.
// ---------------------------------------------------------------------------
template <typename T>
void epToMrpProperties(const Eigen::Vector3<T>& direction, T log10MrpNorm) {
    using Vec3 = Eigen::Vector3<T>;
    using Vec4 = Eigen::Vector4<T>;

    // A zero direction produces the zero MRP (identity rotation).
    Vec3 inputMrp = Vec3::Zero();
    if (!direction.isZero(T(0))) {
        inputMrp = direction.stableNormalized() * std::pow(T(10), log10MrpNorm);
    }

    // Construct Euler parameters from the generated MRP.
    T const inputMrpSquaredNorm = inputMrp.squaredNorm();
    Vec4 ep;
    ep << T(1) - inputMrpSquaredNorm, T(2) * inputMrp;
    ep /= T(1) + inputMrpSquaredNorm;

    // q and -q represent the same attitude.
    Vec4 const negatedEp = -ep;

    T const tol = T(10) * std::numeric_limits<T>::epsilon();
    T const inputMrpNorm = inputMrp.stableNorm();

    // Test both quaternion representations of the same attitude.
    for (Vec4 const& testEp : {ep, negatedEp}) {
        Vec3 const resultMrp = epToMrp(testEp);

        // The resulting MRP must contain only finite values.
        ASSERT_TRUE(resultMrp.allFinite());

        // The principal MRP norm must be at most 1, allowing rounding.
        EXPECT_LE(resultMrp.norm(), T(1) + tol);

        if (inputMrpNorm < T(1) - tol) {
            // Principal MRPs must be recovered unchanged.
            EXPECT_TRUE(resultMrp.isApprox(inputMrp, tol));
        } else if (inputMrpNorm > T(1) + tol) {
            // Non-principal MRPs must be converted to their shadow set.
            EXPECT_TRUE(resultMrp.isApprox(mrpShadow(inputMrp), tol));
        } else {
            // Near 180 degrees, rounding can affect the selected sign.
            EXPECT_TRUE(resultMrp.isApprox(inputMrp, tol) || resultMrp.isApprox(mrpShadow(inputMrp), tol));
        }
    }
}

void epToMrpPropertiesFloat(const Eigen::Vector3f& direction, float log10MrpNorm) {
    epToMrpProperties(direction, log10MrpNorm);
}

void epToMrpPropertiesDouble(const Eigen::Vector3d& direction, double log10MrpNorm) {
    epToMrpProperties(direction, log10MrpNorm);
}

// The Euler parameters are built from MRPs. The largest norms stay below the overflow of |mrp|^2 (|mrp| ~1e19 in
// float, ~1e154 in double). The second range (MRP norms 0.9977 to 1.0023) tests rotations close to 180 deg, where
// q0 changes sign.
FUZZ_TEST(RigidBodyKinematicsFuzz, epToMrpPropertiesFloat)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1.0F, 1.0F),             // Random X, Y, Z direction components.
                 fuzztest::OneOf(fuzztest::InRange(-18.0F, 18.0F),      // MRP norms from 1e-18 to 1e18.
                                 fuzztest::InRange(-0.001F, 0.001F)));  // Rotations from 179.74 to 180.26 deg.

FUZZ_TEST(RigidBodyKinematicsFuzz, epToMrpPropertiesDouble)
    .WithDomains(xmera::fuzz::Vector3dInRange(-1.0, 1.0),             // Random X, Y, Z direction components.
                 fuzztest::OneOf(fuzztest::InRange(-150.0, 150.0),    // MRP norms from 1e-150 to 1e150.
                                 fuzztest::InRange(-0.001, 0.001)));  // Rotations from 179.74 to 180.26 deg.

// ---------------------------------------------------------------------------
// epToPrv: converts Euler parameters to the principal rotation vector, keeping the sign of q0.
// ---------------------------------------------------------------------------
template <typename T>
void epToPrvProperties(const Eigen::Vector3<T>& direction, T log10Angle, T log10EpScale) {
    using Vec3 = Eigen::Vector3<T>;
    using Vec4 = Eigen::Vector4<T>;

    // A zero direction gives a zero vector part, covered by the zero vector part GTest.
    if (direction.isZero(T(0))) {
        return;
    }

    // Euler parameters of the rotation, scaled: q = scale * (cos(phi / 2), sin(phi / 2) * axis).
    Vec3 const axis = direction.stableNormalized();
    T const phi = std::pow(T(10), log10Angle);
    Vec4 ep;
    ep << std::cos(phi / T(2)), std::sin(phi / T(2)) * axis;
    ep *= std::pow(T(10), log10EpScale);

    T const tol = T(10) * std::numeric_limits<T>::epsilon();

    Vec3 const resultPrv = epToPrv(ep);

    // -q describes the same attitude: epToPrv keeps the sign of q0, so it gives the long rotation about -axis.
    Vec4 const negatedEp = -ep;
    Vec3 const resultLongPrv = epToPrv(negatedEp);

    // The rotation vectors must contain only finite values.
    ASSERT_TRUE(resultPrv.allFinite());
    ASSERT_TRUE(resultLongPrv.allFinite());

    // q gives phi * axis. Dividing by the angle compares the result relative to its own size.
    EXPECT_TRUE((resultPrv / phi).isApprox(axis, tol));

    // -q turns about the opposite axis.
    EXPECT_TRUE(resultLongPrv.stableNormalized().isApprox(-axis, tol));

    // The short and long rotation angles add up to a full turn.
    T const twoPi = T(2) * std::numbers::pi_v<T>;
    EXPECT_NEAR(resultPrv.stableNorm() + resultLongPrv.stableNorm(), twoPi, tol * twoPi);
}

void epToPrvPropertiesFloat(const Eigen::Vector3f& direction, float log10Angle, float log10EpScale) {
    epToPrvProperties(direction, log10Angle, log10EpScale);
}

void epToPrvPropertiesDouble(const Eigen::Vector3d& direction, double log10Angle, double log10EpScale) {
    epToPrvProperties(direction, log10Angle, log10EpScale);
}

// The largest angle, 10^0.79 = 6.17 rad, stays below 2 pi. With -q, the long rotations 2 pi - phi cover the rest up
// to 2 pi. The scale keeps the vector part above the smallest normal number (1e-38 in float, 1e-308 in double).
FUZZ_TEST(RigidBodyKinematicsFuzz, epToPrvPropertiesFloat)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1.0F, 1.0F),  // Random X, Y, Z direction components.
                 fuzztest::InRange(-25.0F, 0.79F),           // Rotation angles from 1e-25 to 6.17 rad.
                 fuzztest::InRange(-10.0F, 10.0F));          // Euler parameter norms from 1e-10 to 1e10.

FUZZ_TEST(RigidBodyKinematicsFuzz, epToPrvPropertiesDouble)
    .WithDomains(xmera::fuzz::Vector3dInRange(-1.0, 1.0),  // Random X, Y, Z direction components.
                 fuzztest::InRange(-200.0, 0.79),          // Rotation angles from 1e-200 to 6.17 rad.
                 fuzztest::InRange(-100.0, 100.0));        // Euler parameter norms from 1e-100 to 1e100.

// ---------------------------------------------------------------------------
// dmrp: computes the MRP time derivative from the angular velocity, d(sigma)/dt = 1/4 [B(sigma)] omega.
// ---------------------------------------------------------------------------
template <typename T>
void dmrpProperties(const Eigen::Vector3<T>& mrpDirection,
                    T log10MrpNorm,
                    const Eigen::Vector3<T>& omegaDirection,
                    T log10OmegaNorm) {
    using Vec3 = Eigen::Vector3<T>;

    // A zero direction produces a zero MRP or a zero angular velocity.
    Vec3 mrp = Vec3::Zero();
    if (!mrpDirection.isZero(T(0))) {
        mrp = mrpDirection.stableNormalized() * std::pow(T(10), log10MrpNorm);
    }
    Vec3 omega = Vec3::Zero();
    if (!omegaDirection.isZero(T(0))) {
        omega = omegaDirection.stableNormalized() * std::pow(T(10), log10OmegaNorm);
    }

    Vec3 const mrpRate = dmrp(mrp, omega);
    T const tol = T(10) * std::numeric_limits<T>::epsilon();

    // The MRP rate must contain only finite values.
    ASSERT_TRUE(mrpRate.allFinite());

    // The MRP rate must match the vector form of the MRP kinematic equation,
    // d(sigma)/dt = 1/4 [(1 - |sigma|^2) omega + 2 sigma x omega + 2 sigma (sigma . omega)].
    T const mrpSquaredNorm = mrp.squaredNorm();
    Vec3 const expectedRate =
        ((T(1) - mrpSquaredNorm) * omega + T(2) * mrp.cross(omega) + T(2) * mrp.dot(omega) * mrp) / T(4);
    EXPECT_TRUE(mrpRate.isApprox(expectedRate, tol));

    // [B] [B]^T = (1 + |sigma|^2)^2 I, so the MRP rate has norm (1 + |sigma|^2) |omega| / 4.
    EXPECT_NEAR(mrpRate.stableNorm(), (T(1) + mrpSquaredNorm) * omega.stableNorm() / T(4), tol * mrpRate.stableNorm());
}

void dmrpPropertiesFloat(const Eigen::Vector3f& mrpDirection,
                         float log10MrpNorm,
                         const Eigen::Vector3f& omegaDirection,
                         float log10OmegaNorm) {
    dmrpProperties(mrpDirection, log10MrpNorm, omegaDirection, log10OmegaNorm);
}

void dmrpPropertiesDouble(const Eigen::Vector3d& mrpDirection,
                          double log10MrpNorm,
                          const Eigen::Vector3d& omegaDirection,
                          double log10OmegaNorm) {
    dmrpProperties(mrpDirection, log10MrpNorm, omegaDirection, log10OmegaNorm);
}

// The ranges keep the squared norm of the MRP rate, used by isApprox, finite: |mrp|^2 |omega| stays below 1e13 in
// float and 1e150 in double.
FUZZ_TEST(RigidBodyKinematicsFuzz, dmrpPropertiesFloat)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1.0F, 1.0F),  // Random MRP direction components.
                 fuzztest::InRange(-18.0F, 4.0F),            // MRP norms from 1e-18 to 1e4.
                 xmera::fuzz::Vector3fInRange(-1.0F, 1.0F),  // Random angular velocity direction components.
                 fuzztest::InRange(-10.0F, 5.0F));           // Angular velocity norms from 1e-10 to 1e5 rad/s.

FUZZ_TEST(RigidBodyKinematicsFuzz, dmrpPropertiesDouble)
    .WithDomains(xmera::fuzz::Vector3dInRange(-1.0, 1.0),  // Random MRP direction components.
                 fuzztest::InRange(-150.0, 50.0),          // MRP norms from 1e-150 to 1e50.
                 xmera::fuzz::Vector3dInRange(-1.0, 1.0),  // Random angular velocity direction components.
                 fuzztest::InRange(-50.0, 50.0));          // Angular velocity norms from 1e-50 to 1e50 rad/s.

// ---------------------------------------------------------------------------
// prvToDcm: converts a principal rotation vector to its direction cosine matrix.
// ---------------------------------------------------------------------------
template <typename T>
void prvToDcmProperties(const Eigen::Vector3<T>& direction, T log10Angle) {
    using Vec3 = Eigen::Vector3<T>;
    using Mat3 = Eigen::Matrix<T, 3, 3>;

    // The zero PRV has no rotation axis; it is covered by the identity GTest.
    if (direction.isZero(T(0))) {
        return;
    }

    Vec3 const axis = direction.stableNormalized();
    Vec3 const prv = axis * std::pow(T(10), log10Angle);
    Mat3 const dcm = prvToDcm(prv);
    T const tol = T(10) * std::numeric_limits<T>::epsilon();

    // The DCM must contain only finite values.
    ASSERT_TRUE(dcm.allFinite());

    // The DCM must match the rotation by |prv| about the PRV axis, built independently with Eigen::AngleAxis.
    // AngleAxis gives the active rotation matrix; the DCM is its transpose. A match also makes the DCM orthonormal
    // with determinant +1.
    T const angle = prv.stableNorm();
    Mat3 const expectedDcm = Eigen::AngleAxis<T>(angle, axis).toRotationMatrix().transpose();
    EXPECT_TRUE(dcm.isApprox(expectedDcm, tol));

    // The comparison above is relative to the whole DCM, so it cannot see a small rotation. Below 1 rad, the
    // antisymmetric part, -2 sin(angle) [axis~], carries the rotation and is compared relative to its own size.
    // Closer to 180 deg it becomes small next to the (1 - cos(angle)) terms and loses its relative accuracy.
    if (angle < T(1)) {
        EXPECT_TRUE((dcm - dcm.transpose()).isApprox(expectedDcm - expectedDcm.transpose(), tol));
    }
}

void prvToDcmPropertiesFloat(const Eigen::Vector3f& direction, float log10Angle) {
    prvToDcmProperties(direction, log10Angle);
}

void prvToDcmPropertiesDouble(const Eigen::Vector3d& direction, double log10Angle) {
    prvToDcmProperties(direction, log10Angle);
}

// The first range covers the larger rotations, including 180 and 360 deg; the second covers very small rotations.
FUZZ_TEST(RigidBodyKinematicsFuzz, prvToDcmPropertiesFloat)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1.0F, 1.0F),           // Random X, Y, Z direction components.
                 fuzztest::OneOf(fuzztest::InRange(-6.0F, 1.0F),      // Angles from 1e-6 to 10 rad.
                                 fuzztest::InRange(-18.0F, -6.0F)));  // Angles from 1e-18 to 1e-6 rad.

FUZZ_TEST(RigidBodyKinematicsFuzz, prvToDcmPropertiesDouble)
    .WithDomains(xmera::fuzz::Vector3dInRange(-1.0, 1.0),            // Random X, Y, Z direction components.
                 fuzztest::OneOf(fuzztest::InRange(-6.0, 1.0),       // Angles from 1e-6 to 10 rad.
                                 fuzztest::InRange(-150.0, -6.0)));  // Angles from 1e-150 to 1e-6 rad.

// ---------------------------------------------------------------------------
// subMrp: computes the MRP set of frame 1 relative to frame 2.
// ---------------------------------------------------------------------------
template <typename T>
void subMrpProperties(const Eigen::Vector3<T>& mrp1, const Eigen::Vector3<T>& mrp2) {
    using Vec3 = Eigen::Vector3<T>;
    using Mat3 = Eigen::Matrix3<T>;

    Vec3 const resultMrp = subMrp(mrp1, mrp2);

    // The result must contain only finite values.
    ASSERT_TRUE(resultMrp.allFinite());

    // The result is the principal MRP set, norm at most 1.
    EXPECT_LE(resultMrp.norm(), T(1));

    // The DCM of the result must be [C1] [C2]^T. The DCM of each input is built independently with
    // Eigen::AngleAxis, using the rotation angle 4 atan(|mrp|) about the MRP axis (the DCM is the transpose of its
    // matrix). The comparison has three DCMs, each within 10 eps, so the two sides can differ by up to 30 eps.
    auto const toDcm = [](Vec3 const& mrp) -> Mat3 {
        return Eigen::AngleAxis<T>(T(4) * std::atan(mrp.stableNorm()), mrp.stableNormalized())
            .toRotationMatrix()
            .transpose();
    };
    T const tol = T(30) * std::numeric_limits<T>::epsilon();
    EXPECT_TRUE(mrpToDcm(resultMrp).isApprox(toDcm(mrp1) * toDcm(mrp2).transpose(), tol));
}

void subMrpPropertiesFloat(const Eigen::Vector3f& mrp1, const Eigen::Vector3f& mrp2) { subMrpProperties(mrp1, mrp2); }

void subMrpPropertiesDouble(const Eigen::Vector3d& mrp1, const Eigen::Vector3d& mrp2) { subMrpProperties(mrp1, mrp2); }

// MRP norms below 1 are rotations up to 180 deg; norms above 1 are the non-principal sets of rotations above 180 deg.
// The domain therefore covers the shadow branch and the switch to the principal set.
FUZZ_TEST(RigidBodyKinematicsFuzz, subMrpPropertiesFloat)
    .WithDomains(xmera::fuzz::Vector3fInRange(-2.0F, 2.0F),   // Random MRP 1 components.
                 xmera::fuzz::Vector3fInRange(-2.0F, 2.0F));  // Random MRP 2 components.

FUZZ_TEST(RigidBodyKinematicsFuzz, subMrpPropertiesDouble)
    .WithDomains(xmera::fuzz::Vector3dInRange(-2.0, 2.0),   // Random MRP 1 components.
                 xmera::fuzz::Vector3dInRange(-2.0, 2.0));  // Random MRP 2 components.

// ---------------------------------------------------------------------------
// addMrp: computes the MRP set of two successive rotations, first mrp1, then mrp2.
// ---------------------------------------------------------------------------
template <typename T>
void addMrpProperties(const Eigen::Vector3<T>& mrp1, const Eigen::Vector3<T>& mrp2) {
    using Vec3 = Eigen::Vector3<T>;
    using Mat3 = Eigen::Matrix3<T>;

    Vec3 const resultMrp = addMrp(mrp1, mrp2);

    // The result must contain only finite values.
    ASSERT_TRUE(resultMrp.allFinite());

    // The result is the principal MRP set, norm at most 1.
    EXPECT_LE(resultMrp.norm(), T(1));

    // The first rotation is mrp1 and the second is mrp2, so the DCM of the result must be [C2] [C1]. The DCM of each
    // input is built independently with Eigen::AngleAxis, using the rotation angle 4 atan(|mrp|) about the MRP axis
    // (the DCM is the transpose of its matrix). The comparison has three DCMs, each within 10 eps, so the two sides can
    // differ by up to 30 eps.
    auto const toDcm = [](Vec3 const& mrp) -> Mat3 {
        return Eigen::AngleAxis<T>(T(4) * std::atan(mrp.stableNorm()), mrp.stableNormalized())
            .toRotationMatrix()
            .transpose();
    };
    T const tol = T(30) * std::numeric_limits<T>::epsilon();
    EXPECT_TRUE(mrpToDcm(resultMrp).isApprox(toDcm(mrp2) * toDcm(mrp1), tol));
}

void addMrpPropertiesFloat(const Eigen::Vector3f& mrp1, const Eigen::Vector3f& mrp2) { addMrpProperties(mrp1, mrp2); }

void addMrpPropertiesDouble(const Eigen::Vector3d& mrp1, const Eigen::Vector3d& mrp2) { addMrpProperties(mrp1, mrp2); }

// MRP norms below 1 are rotations up to 180 deg; norms above 1 are the non-principal sets of rotations above 180 deg.
// The domain therefore covers the shadow branch and the switch to the principal set.
FUZZ_TEST(RigidBodyKinematicsFuzz, addMrpPropertiesFloat)
    .WithDomains(xmera::fuzz::Vector3fInRange(-2.0F, 2.0F),   // Random MRP 1 components.
                 xmera::fuzz::Vector3fInRange(-2.0F, 2.0F));  // Random MRP 2 components.

FUZZ_TEST(RigidBodyKinematicsFuzz, addMrpPropertiesDouble)
    .WithDomains(xmera::fuzz::Vector3dInRange(-2.0, 2.0),   // Random MRP 1 components.
                 xmera::fuzz::Vector3dInRange(-2.0, 2.0));  // Random MRP 2 components.
}  // namespace
