#ifndef F32XMERA_CONVERT_ST_PLATFORM_TO_BODY_ALGORITHM_H
#define F32XMERA_CONVERT_ST_PLATFORM_TO_BODY_ALGORITHM_H

#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/rigidBodyKinematics.hpp"

#include <Eigen/Core>

/*! @brief Body-frame attitude output from the platform-to-body conversion algorithm. */
struct StAttitudeOutput {
    Eigen::Vector3f sigma_BN = Eigen::Vector3f::Zero();    //!< [-] MRP from inertial to body frame
    Eigen::Vector3f omega_BN_B = Eigen::Vector3f::Zero();  //!< [rad/s] body-frame angular velocity w.r.t. inertial
};

/*!
 * @brief Validated configuration for the platform-to-body conversion algorithm.
 *
 * An instance can only exist with a finite MRP of the platform frame relative to the body frame.
 * Construct via ConvertStPlatformToBodyConfig::create(...).
 */
class ConvertStPlatformToBodyConfig final {
   public:
    static ConvertStPlatformToBodyConfig create(const Eigen::Vector3f& sigma_PB) {
        if (!isValidSigma_PB(sigma_PB)) {
            FSW_THROW_INVALID_ARGUMENT("convertStPlatformToBody: sigma_PB must be finite");
        }
        // The shadow set describes the same rotation, so store the principal one (norm <= 1).
        return ConvertStPlatformToBodyConfig{mrpSwitch(sigma_PB)};
    }

    static bool isValidSigma_PB(const Eigen::Vector3f& sigma_PB) { return sigma_PB.allFinite(); }

    const Eigen::Vector3f& getSigma_PB() const { return sigma_PB; }

   private:
    explicit ConvertStPlatformToBodyConfig(const Eigen::Vector3f& sigma_PB) : sigma_PB(sigma_PB) {}

    Eigen::Vector3f sigma_PB;
};

class ConvertStPlatformToBodyAlgorithm final {
   public:
    explicit ConvertStPlatformToBodyAlgorithm(const ConvertStPlatformToBodyConfig& config);
    void setConfig(const ConvertStPlatformToBodyConfig& config);

    //! @param q_PN  inertial-to-platform attitude quaternion (scalar-last, sensor native)
    //! @param dq_PN platform-frame angular velocity quaternion (scalar-last, see module documentation)
    StAttitudeOutput update(const Eigen::Vector4f& q_PN, const Eigen::Vector4f& dq_PN) const;

   private:
    ConvertStPlatformToBodyConfig cfg;
    Eigen::Vector3f sigma_BP{Eigen::Vector3f::Zero()};    //!< [-] MRP of the body frame B relative to platform frame P
    Eigen::Matrix3f dcm_BP{Eigen::Matrix3f::Identity()};  //!< [-] DCM from the platform frame P to the body frame B
};

#endif
