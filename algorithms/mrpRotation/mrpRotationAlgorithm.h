#ifndef F32XMERA_MRP_ROTATION_ALGORITHM_H
#define F32XMERA_MRP_ROTATION_ALGORITHM_H

#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/freestandingIsFinite.hpp"
#include "utilities/fsw/rigidBodyKinematics.hpp"

#include <Eigen/Core>

/*! @brief Algorithm-native input bundle that mirrors AttRefMsgF32Payload. The adapter converts the
 *  payload's float[3] arrays into Eigen vectors via eigenSupport.h before calling update().
 */
struct MrpRotationAttRefInputs {
    Eigen::Vector3f sigma_R0N{Eigen::Vector3f::Zero()};     //!< [-] input ref MRP attitude wrt inertial N
    Eigen::Vector3f omega_R0N_N{Eigen::Vector3f::Zero()};   //!< [rad/s] input ref angular velocity, N-frame components
    Eigen::Vector3f domega_R0N_N{Eigen::Vector3f::Zero()};  //!< [rad/s^2] input ref angular acceleration, N-frame
};

/*! @brief Algorithm-native output bundle shaped like AttRefMsgF32Payload. The adapter packs these
 *  Eigen fields back into the payload via eigenSupport.h.
 */
struct MrpRotationOutput {
    Eigen::Vector3f sigma_RN{Eigen::Vector3f::Zero()};     //!< [-] output ref MRP wrt inertial N
    Eigen::Vector3f omega_RN_N{Eigen::Vector3f::Zero()};   //!< [rad/s] output ref angular velocity, N-frame
    Eigen::Vector3f domega_RN_N{Eigen::Vector3f::Zero()};  //!< [rad/s^2] output ref angular acceleration, N-frame
};

class MrpRotationConfig final {
   public:
    static MrpRotationConfig create(const Eigen::Vector3f& sigma_RcR0,
                                    const Eigen::Vector3f& sigma_RpRc,
                                    const Eigen::Vector3f& omega_RRp_R,
                                    float controlPeriod) {
        if (!isValidSigma_RcR0(sigma_RcR0)) {
            FSW_THROW_INVALID_ARGUMENT("mrpRotation: sigma_RcR0 must be finite");
        }
        if (!isValidSigma_RpRc(sigma_RpRc)) {
            FSW_THROW_INVALID_ARGUMENT("mrpRotation: sigma_RpRc must be finite");
        }
        if (!isValidOmega_RRp_R(omega_RRp_R)) {
            FSW_THROW_INVALID_ARGUMENT("mrpRotation: omega_RRp_R must be finite");
        }
        if (!isValidControlPeriod(controlPeriod)) {
            FSW_THROW_INVALID_ARGUMENT("mrpRotation: controlPeriod must be > 0");
        }
        return {mrpSwitch(sigma_RcR0), mrpSwitch(sigma_RpRc), omega_RRp_R, controlPeriod};
    }

    static bool isValidSigma_RcR0(const Eigen::Vector3f& sigma) { return sigma.allFinite(); }
    static bool isValidSigma_RpRc(const Eigen::Vector3f& sigma) { return sigma.allFinite(); }
    static bool isValidOmega_RRp_R(const Eigen::Vector3f& omega) { return omega.allFinite(); }
    static bool isValidControlPeriod(float period) { return fsw::is_finite(period) && period > 0.0F; }

    const Eigen::Vector3f& getSigma_RcR0() const { return this->sigma_RcR0; }
    const Eigen::Vector3f& getSigma_RpRc() const { return this->sigma_RpRc; }
    const Eigen::Vector3f& getOmega_RRp_R() const { return this->omega_RRp_R; }
    float getControlPeriod() const { return this->controlPeriod; }

   private:
    MrpRotationConfig(const Eigen::Vector3f& sigma_RcR0,
                      const Eigen::Vector3f& sigma_RpRc,
                      const Eigen::Vector3f& omega_RRp_R,
                      float controlPeriod)
        : sigma_RcR0(sigma_RcR0), sigma_RpRc(sigma_RpRc), omega_RRp_R(omega_RRp_R), controlPeriod(controlPeriod) {}

    Eigen::Vector3f sigma_RcR0;   //!< [-] fixed body correction: corrected frame Rc wrt input reference R0
    Eigen::Vector3f sigma_RpRc;   //!< [-] commanded pointing offset: frame Rp wrt Rc, in body axes
    Eigen::Vector3f omega_RRp_R;  //!< [rad/s] constant rotation rate of R wrt Rp, in output (body) axes
    float controlPeriod;          //!< [s] forward-Euler integration step
};

/*! @brief MRP Rotation algorithm class. */
class MrpRotationAlgorithm final {
   public:
    explicit MrpRotationAlgorithm(const MrpRotationConfig& config);

    void setConfig(const MrpRotationConfig& config);

    void reInitialize();

    MrpRotationOutput update(const MrpRotationAttRefInputs& attRef);

   private:
    MrpRotationConfig cfg;

    Eigen::Matrix3f dcm_RpR0{Eigen::Matrix3f::Identity()};  //!< [-] cached [RpRc][RcR0], set by setConfig
    Eigen::Vector3f sigma_RRp{Eigen::Vector3f::Zero()};     //!< [-] integrated rotation of R wrt Rp
};

#endif
