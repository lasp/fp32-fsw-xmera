#include "../validDcmCheck.h"
#include "../validInertiaCheck.h"

#include <gtest/gtest.h>
#include <Eigen/Core>

/* Function to generate a cross product operator from a vector (double precision) */
inline Eigen::Matrix3d tildeMatrix(const Eigen::Vector3d& vector) {
    Eigen::Matrix3d tilde;
    tilde << 0.0, -vector(2), vector(1), vector(2), 0.0, -vector(0), -vector(1), vector(0), 0.0;
    return tilde;
}

/* Function to generate a dcm from an mrp (double precision) */
inline Eigen::Matrix3d mrpToDcm(const Eigen::Vector3d& mrp) {
    const Eigen::Matrix3d t = tildeMatrix(mrp);
    const double denom = (1.0 + mrp.dot(mrp));
    const Eigen::Matrix3d dcm = (8.0 * t * t - 4.0 * (1.0 - mrp.dot(mrp)) * t) / (denom * denom);

    return dcm + Eigen::Matrix3d::Identity();
}

/* Function to generate a general inertia matrix using double precision internally */
inline Eigen::Matrix3d generateValidInertiaMatrix(double const ev1,
                                                  double const ev2,
                                                  double const sigma1,
                                                  double const sigma2,
                                                  double const sigma3) {
    /* Sort the first two eigenvalues so ev_min <= ev_max */
    const double ev_min = std::min(ev1, ev2);
    const double ev_max = std::max(ev1, ev2);
    const double ev3_min = (ev_max - ev_min > 1e-5) ? ev_max - ev_min + 1e-5 : 1e-5;
    const double ev3_max = ev_max + ev_min - 1e-5;

    const double ev3 = ev3_min + (0.5 * (ev3_max - ev3_min));

    /* Create a rotation matrix with provided mrp (double precision) */
    Eigen::Vector3d mrp(static_cast<double>(sigma1), static_cast<double>(sigma2), static_cast<double>(sigma3));
    if (mrp.squaredNorm() > 1) {
        mrp = -mrp / mrp.squaredNorm();
    }
    Eigen::Matrix3d R = mrpToDcm(mrp);

    /* Create diagonal matrix D with eigenvalues */
    Eigen::Matrix3d D;
    D << ev_min, 0, 0, 0, ev3, 0, 0, 0, ev_max;

    /* Return rotated diagonal matrix, cast to float */
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
