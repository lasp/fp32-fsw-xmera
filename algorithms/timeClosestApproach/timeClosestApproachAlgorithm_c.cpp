#include "timeClosestApproachAlgorithm_c.h"
#include "timeClosestApproachAlgorithm.h"
#include "utilities/fsw/opaqueHandle.h"

#include <Eigen/Core>

uint32_t TimeClosestApproachAlgorithm_getMaxFilterStates(void) { return 6U; }

TimeClosestApproachAlgorithmHandle* TimeClosestApproachAlgorithm_create(void) {
    return fsw::createHandle<::TimeClosestApproachAlgorithm, TimeClosestApproachAlgorithmHandle>();
}

void TimeClosestApproachAlgorithm_destroy(TimeClosestApproachAlgorithmHandle* self) {
    fsw::deleteHandle<::TimeClosestApproachAlgorithm>(self);
}

TimeClosestApproachOutput_c TimeClosestApproachAlgorithm_update(const TimeClosestApproachAlgorithmHandle* self,
                                                                const Vector3d_c* r_BN_N,
                                                                const Vector3d_c* v_BN_N,
                                                                const FilterCovariance_c* covariance) {
    const Eigen::Vector3d r = Eigen::Map<const Eigen::Vector3d>(r_BN_N->data);
    const Eigen::Vector3d v = Eigen::Map<const Eigen::Vector3d>(v_BN_N->data);
    const Eigen::Matrix<double, 6, 6> P = Eigen::Map<const Eigen::Matrix<float, 6, 6>>(covariance->data).cast<double>();

    const TimeClosestApproachOutput out = fsw::fromHandle<const ::TimeClosestApproachAlgorithm>(self)->update(r, v, P);

    TimeClosestApproachOutput_c result{};
    result.tCA = out.tCA;
    result.sigmaTca = out.sigmaTca;
    return result;
}
