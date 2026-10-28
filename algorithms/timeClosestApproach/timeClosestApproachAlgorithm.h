#ifndef F32XMERA_TIME_CA_ALGORITHM_H
#define F32XMERA_TIME_CA_ALGORITHM_H

#include "timeClosestApproachTypes.h"
#include <Eigen/Core>

/** @brief Minimum Euclidean norm [m or m/s] for r_BN_N and v_BN_N inputs. */
inline constexpr float kMinVectorNorm = 1.0e-3F;

struct TimeClosestApproachOutput {
    float tCA;       //!< the predicted time of closest approach [s]
    float sigmaTca;  //!< the predicted time of closest approach standard deviation [s]
};

class TimeClosestApproachAlgorithm final {
   public:
    static TimeClosestApproachOutput update(const Eigen::Vector3f& r_BN_N,
                                            const Eigen::Vector3f& v_BN_N,
                                            const Eigen::Matrix<float, 6, 6>& filterCovariance);
};

#endif
