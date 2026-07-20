#ifndef F32XMERA_TIME_CA_ALGORITHM_H
#define F32XMERA_TIME_CA_ALGORITHM_H

#include "timeClosestApproachTypes.h"
#include <Eigen/Core>

/** @brief Minimum Euclidean norm [m or m/s] for r_BN_N and v_BN_N inputs. */
inline constexpr double kMinVectorNorm = 1.0e-3;

struct TimeClosestApproachOutput {
    float tCA;       //!< the predicted time of closest approach [s]
    float sigmaTca;  //!< the predicted time of closest approach standard deviation [s]
};

class TimeClosestApproachAlgorithm final {
   public:
    static TimeClosestApproachOutput update(const Eigen::Vector3d& r_BN_N,
                                            const Eigen::Vector3d& v_BN_N,
                                            const Eigen::Matrix<double, 6, 6>& filterCovariance);
};

#endif
