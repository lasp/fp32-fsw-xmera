#ifndef F32XMERA_NAV_AGGREGATE_OUTPUT_H
#define F32XMERA_NAV_AGGREGATE_OUTPUT_H

#include <Eigen/Core>

#ifdef __cplusplus
extern "C" {
#endif

/*! Struct containing the attitude navigation input needed by the algorithm. */
struct InputNavAttData {
    double timeTag{};
    Eigen::Vector3f sigma_BN = Eigen::Vector3f::Zero();
    Eigen::Vector3f omega_BN_B = Eigen::Vector3f::Zero();
    Eigen::Vector3f vehSunPntBdy = Eigen::Vector3f::Zero();
};

/*! Struct containing the translational navigation input needed by the algorithm. */
struct InputNavTransData {
    double timeTag{};
    Eigen::Vector3d r_BN_N = Eigen::Vector3d::Zero();
    Eigen::Vector3d v_BN_N = Eigen::Vector3d::Zero();
    Eigen::Vector3f vehAccumDV = Eigen::Vector3f::Zero();
};

/*! structure containing the attitude and translational navigation outputs */
typedef struct {
    InputNavAttData navAttOut;     /*!< attitude navigation output */
    InputNavTransData navTransOut; /*!< translation navigation output */
} AggregateOutput;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_NAV_AGGREGATE_OUTPUT_H
