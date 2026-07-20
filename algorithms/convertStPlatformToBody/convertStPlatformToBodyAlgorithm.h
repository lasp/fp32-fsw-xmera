#ifndef F32XIMERA_CONVERT_ST_PLATFORM_TO_BODY_ALGORITHM_H
#define F32XIMERA_CONVERT_ST_PLATFORM_TO_BODY_ALGORITHM_H

#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/validDcmCheck.h"

#include <Eigen/Core>

/*! @brief Body-frame attitude output from the platform-to-body conversion algorithm. */
struct StAttitudeOutput {
    Eigen::Vector3f sigma_BN = Eigen::Vector3f::Zero();    //!< [-] MRP from inertial to body frame
    Eigen::Vector3f omega_BN_B = Eigen::Vector3f::Zero();  //!< [rad/s] body-frame angular velocity w.r.t. inertial
};

/*!
 * @brief Validated configuration for the platform-to-body conversion algorithm.
 *
 * An instance can only exist with a valid (orthonormal, det +1) body-to-case mounting DCM.
 * Construct via ConvertStPlatformToBodyConfig::create(...).
 */
class ConvertStPlatformToBodyConfig final {
   public:
    static ConvertStPlatformToBodyConfig create(const Eigen::Matrix3f& dcm_CB) {
        if (!isValidDcmCB(dcm_CB)) {
            FSW_THROW_INVALID_ARGUMENT("convertStPlatformToBody: dcm_CB must be a valid DCM (orthonormal, det +1)");
        }
        return ConvertStPlatformToBodyConfig{dcm_CB};
    }

    static bool isValidDcmCB(const Eigen::Matrix3f& dcm_CB) { return isValidDcm(dcm_CB); }

    const Eigen::Matrix3f& getDcmCB() const { return dcm_CB; }

   private:
    explicit ConvertStPlatformToBodyConfig(const Eigen::Matrix3f& dcm_CB) : dcm_CB(dcm_CB) {}

    Eigen::Matrix3f dcm_CB;
};

class ConvertStPlatformToBodyAlgorithm final {
   public:
    explicit ConvertStPlatformToBodyAlgorithm(const ConvertStPlatformToBodyConfig& config);
    void setConfig(const ConvertStPlatformToBodyConfig& config);
    //! @param q_CN  inertial-to-case attitude quaternion (scalar-first)
    //! @param dq_CN case-frame delta quaternion (scalar-last)
    StAttitudeOutput update(const Eigen::Vector4f& q_CN, const Eigen::Vector4f& dq_CN) const;

   private:
    ConvertStPlatformToBodyConfig cfg;
};

#endif
