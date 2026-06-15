#ifndef F32XIMERA_CONVERT_ST_PLATFORM_TO_BODY_ALGORITHM_H
#define F32XIMERA_CONVERT_ST_PLATFORM_TO_BODY_ALGORITHM_H

#include "convertStPlatformToBodyTypes.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/validDcmCheck.h"

#include <Eigen/Core>

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

class ConvertStPlatformToBodyAlgorithm {
   public:
    StAttitudeOutput update(const PlatformAttitude& platformAttitude,
                            const PlatformAngularVelocity& platformAngularRate) const;
    void setDcmCB(const Eigen::Matrix3f& dcm_CB);
    const Eigen::Matrix3f& getDcmCB() const;

   private:
    Eigen::Matrix3f dcm_CB{Eigen::Matrix3f::Identity()};
};

#endif
