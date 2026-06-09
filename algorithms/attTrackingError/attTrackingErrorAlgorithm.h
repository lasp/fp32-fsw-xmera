#ifndef F32XMERA_ATT_TRACKING_ERROR_ALGORITHM_H
#define F32XMERA_ATT_TRACKING_ERROR_ALGORITHM_H

#include <Eigen/Core>

/*! @brief Structure containing the attitude navigation input needed by the algorithm */
struct AttNavInput {
    Eigen::Vector3f sigma_BN = Eigen::Vector3f::Zero();
    Eigen::Vector3f omega_BN_B = Eigen::Vector3f::Zero();
};

/*! @brief Structure containing the attitude reference input needed by the algorithm */
struct AttRefInput {
    Eigen::Vector3f sigma_RN = Eigen::Vector3f::Zero();
    Eigen::Vector3f omega_RN_N = Eigen::Vector3f::Zero();
    Eigen::Vector3f domega_RN_N = Eigen::Vector3f::Zero();
};

/*! structure containing the attitude guidance outputs of the algorithm */
struct AttGuidOutput {
    Eigen::Vector3f sigma_BR = Eigen::Vector3f::Zero();
    Eigen::Vector3f omega_BR_B = Eigen::Vector3f::Zero();
    Eigen::Vector3f omega_RN_B = Eigen::Vector3f::Zero();
    Eigen::Vector3f domega_RN_B = Eigen::Vector3f::Zero();
};

class AttTrackingErrorAlgorithm {
   public:
    AttGuidOutput update(const AttNavInput& navIn, const AttRefInput& refIn);  //!< Algorithm update method
};

#endif
