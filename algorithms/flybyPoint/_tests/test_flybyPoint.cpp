#include "flybyPointTestHelpers.hpp"

TEST(flybyPointTest, ConfigValidation) { configValidationTest(); }

TEST(flybyPointTest, RegressionTest) {
    regressionTestFlybyPoint(60.0,     // timeBetweenFilterData
                             0.01F,    // toleranceForCollinearity
                             1,        // signOfOrbitNormalFrameVector
                             5.0F,     // maxRateThreshold
                             2.0F,     // maxAccelerationThreshold
                             1000.0F,  // positionKnowledgeSigma
                             Eigen::Vector3d{5000.0, 0.0, 10000.0},
                             Eigen::Vector3d{0.0, 500.0, 300.0});
}

TEST(flybyPointTest, ValidOutputSeeded) {
    validOutputSeededTest(Eigen::Vector3d{5000.0, 0.0, 10000.0}, Eigen::Vector3d{0.0, 500.0, 300.0});
}

TEST(flybyPointTest, CollinearRejection) { collinearRejectionTest(); }
