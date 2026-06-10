#include "timeClosestApproachAlgorithm_c.h"
#include "timeClosestApproachAlgorithm.h"

#include <Eigen/Core>

uint32_t TimeClosestApproachAlgorithm_getMaxFilterStates(void) { return TIME_CA_MAX_FILTER_STATES; }

TimeClosestApproachAlgorithmHandle* TimeClosestApproachAlgorithm_create(void) {
    return reinterpret_cast<TimeClosestApproachAlgorithmHandle*>(
        new ::TimeClosestApproachAlgorithm(TimeClosestApproachConfig::create()));
}

void TimeClosestApproachAlgorithm_destroy(TimeClosestApproachAlgorithmHandle* self) {
    delete reinterpret_cast<::TimeClosestApproachAlgorithm*>(self);
}

TimeClosestApproachOutput_c TimeClosestApproachAlgorithm_update(const TimeClosestApproachAlgorithmHandle* self,
                                                                const Vector3d_c* r_BN_N,
                                                                const Vector3d_c* v_BN_N,
                                                                const FilterCovariance_c* covariance) {
    const Eigen::Vector3d r = Eigen::Map<const Eigen::Vector3d>(r_BN_N->data);
    const Eigen::Vector3d v = Eigen::Map<const Eigen::Vector3d>(v_BN_N->data);
    const Eigen::Matrix<float, 6, 6> P = Eigen::Map<const Eigen::Matrix<float, 6, 6>>(covariance->data);

    const TimeClosestApproachOutput out =
        reinterpret_cast<const ::TimeClosestApproachAlgorithm*>(self)->update(r, v, P);

    TimeClosestApproachOutput_c result{};
    result.tCA = out.tCA;
    result.sigmaTca = out.sigmaTca;
    return result;
}
