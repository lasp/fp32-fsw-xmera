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
 * An instance can only exist with a finite MRP of the case frame relative to the body frame.
 * Construct via ConvertStPlatformToBodyConfig::create(...).
 */
class ConvertStPlatformToBodyConfig final {
   public:
    static ConvertStPlatformToBodyConfig create(const Eigen::Vector3f& sigma_CB) {
        if (!isValidSigma_CB(sigma_CB)) {
            FSW_THROW_INVALID_ARGUMENT("convertStPlatformToBody: sigma_CB must be finite");
        }
        // The shadow set describes the same rotation, so store the principal one (norm <= 1).
        return ConvertStPlatformToBodyConfig{mrpSwitch(sigma_CB)};
    }

    static bool isValidSigma_CB(const Eigen::Vector3f& sigma_CB) { return sigma_CB.allFinite(); }

    const Eigen::Vector3f& getSigma_CB() const { return sigma_CB; }

   private:
    explicit ConvertStPlatformToBodyConfig(const Eigen::Vector3f& sigma_CB) : sigma_CB(sigma_CB) {}

    Eigen::Vector3f sigma_CB;
};

class ConvertStPlatformToBodyAlgorithm final {
   public:
    explicit ConvertStPlatformToBodyAlgorithm(const ConvertStPlatformToBodyConfig& config);
    void setConfig(const ConvertStPlatformToBodyConfig& config);

    //! @param q_CN  inertial-to-case attitude quaternion (scalar-last, sensor native)
    //! @param dq_CN case-frame angular velocity quaternion (scalar-last, see module documentation)
    StAttitudeOutput update(const Eigen::Vector4f& q_CN, const Eigen::Vector4f& dq_CN) const;

   private:
    ConvertStPlatformToBodyConfig cfg;
    Eigen::Vector3f sigma_BC{Eigen::Vector3f::Zero()};    //!< [-] MRP of the body frame B relative to the case frame C
    Eigen::Matrix3f dcm_BC{Eigen::Matrix3f::Identity()};  //!< [-] DCM from the case frame C to the body frame B
};

#endif
