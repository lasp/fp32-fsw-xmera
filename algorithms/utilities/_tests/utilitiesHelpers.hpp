#include "../validDcmCheck.h"
#include "../validInertiaCheck.h"

#include <gtest/gtest.h>
#include <Eigen/Core>

/* Function to generate a cross product operator from a vector */
inline Eigen::Matrix3f tildeMatrix(const Eigen::Vector3f& vector) {
    Eigen::Matrix3f tilde;
    tilde << 0.0F, -vector(2), vector(1), vector(2), 0.0F, -vector(0), -vector(1), vector(0), 0.0F;
    return tilde;
}

/* Function to generate a dcm from an mrp */
inline Eigen::Matrix3f mrpToDcm(const Eigen::Vector3f& mrp) {
    const Eigen::Matrix3f t = tildeMatrix(mrp);
    const float denom = (1.0 + mrp.dot(mrp));
    const Eigen::Matrix3f dcm = (8.0 * t * t - 4.0 * (1.0 - mrp.dot(mrp)) * t) / (denom * denom);

    return dcm + Eigen::Matrix3f::Identity();
}

/* Function to generate a general inertia matrix */
inline Eigen::Matrix3f GenerateValidInertiaMatrix(float const ev1,
                                                  float const ev2,
                                                  float const sigma1,
                                                  float const sigma2,
                                                  float const sigma3) {
    /* Sort the first two eigenvalues so ev_min <= ev_max,
       and clamp the ratio to avoid float32 precision loss
       in the R^T * D * R round-trip. */
    constexpr float maxConditionNumber = 1e4f;
    const float ev_max = std::max(ev1, ev2);
    const float ev_min = std::max(std::min(ev1, ev2), ev_max / maxConditionNumber);
    constexpr float triangleInequalityTolerance = 1e-5f;
    const float ev3_min = (ev_max - ev_min > triangleInequalityTolerance)
                              ? ev_max - ev_min + triangleInequalityTolerance
                              : triangleInequalityTolerance;
    const float ev3_max = ev_max + ev_min - triangleInequalityTolerance;

    const float ev3 = ev3_min + (0.5 * (ev3_max - ev3_min));
    const std::array<float, 3> eigenvalues = {ev_min, ev3, ev_max};

    /* Create a rotation matrix with provided mrp */
    Eigen::Vector3f mrp(sigma1, sigma2, sigma3);
    if (mrp.squaredNorm() > 1) {
        mrp = -mrp / mrp.squaredNorm();
    }
    Eigen::Matrix3f R = mrpToDcm(mrp);

    /* Create diagonal matrix D with eigenvalues */
    Eigen::Matrix3f D;
    D << eigenvalues[0], 0, 0, 0, eigenvalues[1], 0, 0, 0, eigenvalues[2];

    /* Return rotated diagonal matrix */
    return R.transpose() * D * R;
}

/* Test validity against generated inertia matrix */
inline void testInertiaValidity(float eigen1, float eigen2, float sigma1, float sigma2, float sigma3) {
    const Eigen::Matrix3f inertia = GenerateValidInertiaMatrix(eigen1, eigen2, sigma1, sigma2, sigma3);

    EXPECT_TRUE(inertiaIsValid(inertia));
}

/* Function to generate a general inertia matrix */
inline Eigen::Matrix3f generateValidDCM(float const sigma1, float const sigma2, float const sigma3) {
    Eigen::Vector3f mrp(sigma1, sigma2, sigma3);
    if (mrp.squaredNorm() > 1) {
        mrp = -mrp / mrp.squaredNorm();
    }
    Eigen::Matrix3f dcm = mrpToDcm(mrp);

    return dcm;
}

inline void testDcmValidity(float const sigma1, float const sigma2, float const sigma3) {
    const Eigen::Matrix3f dcm = generateValidDCM(sigma1, sigma2, sigma3);

    EXPECT_TRUE(isValidDcm(dcm));
}
