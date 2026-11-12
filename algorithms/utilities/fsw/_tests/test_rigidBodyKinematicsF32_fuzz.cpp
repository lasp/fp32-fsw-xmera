#include "utilities/fsw/rigidBodyKinematics.hpp"
#include "utilities/testUtilities/eigenFuzzDomains.hpp"

#include <fuzztest/fuzztest.h>
#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>
#include <limits>
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
}  // namespace
