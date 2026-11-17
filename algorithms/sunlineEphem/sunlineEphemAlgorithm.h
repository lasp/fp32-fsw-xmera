#ifndef F32XMERA_SUNLINE_EPHEM_ALGORITHM_H
#define F32XMERA_SUNLINE_EPHEM_ALGORITHM_H

#include <Eigen/Core>

/*! @brief Sun direction relative to the spacecraft, in the inertial and the body frame. */
struct SunlineEphemOutput {
    Eigen::Vector3f rHat_SB_N{Eigen::Vector3f::Zero()};  //!< [-] unit Sun direction, inertial frame
    Eigen::Vector3f rHat_SB_B{Eigen::Vector3f::Zero()};  //!< [-] unit Sun direction, body frame
};

class SunlineEphemAlgorithm final {
   public:
    static SunlineEphemOutput update(const Eigen::Vector3d& r_SN_N,
                                     const Eigen::Vector3d& r_BN_N,
                                     const Eigen::Vector3f& sigma_BN);
};

#endif
