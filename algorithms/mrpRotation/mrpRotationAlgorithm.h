#ifndef F32XMERA_MRP_ROTATION_ALGORITHM_H
#define F32XMERA_MRP_ROTATION_ALGORITHM_H

#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/freestandingIsFinite.hpp"
#include "utilities/fsw/rigidBodyKinematics.hpp"

#include <math.h>
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
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
    static MrpRotationConfig create(const Eigen::Vector3f& sigma_RbR0,
                                    const Eigen::Vector3f& sigma_RpRb,
                                    const Eigen::Vector3f& spinAxis_Rp,
                                    float spinRate,
                                    float controlPeriod) {
        if (!isValidSigma_RbR0(sigma_RbR0)) {
            FSW_THROW_INVALID_ARGUMENT("mrpRotation: sigma_RbR0 must be finite");
        }
        if (!isValidSigma_RpRb(sigma_RpRb)) {
            FSW_THROW_INVALID_ARGUMENT("mrpRotation: sigma_RpRb must be finite");
        }
        if (!isValidSpinAxis_Rp(spinAxis_Rp)) {
            FSW_THROW_INVALID_ARGUMENT("mrpRotation: spinAxis_Rp must be finite with norm within 1e-3 of 1.0");
        }
        if (!isValidSpinRate(spinRate)) {
            FSW_THROW_INVALID_ARGUMENT("mrpRotation: spinRate must be finite");
        }
        if (!isValidControlPeriod(controlPeriod)) {
            FSW_THROW_INVALID_ARGUMENT("mrpRotation: controlPeriod must be > 0");
        }
        return {mrpSwitch(sigma_RbR0), mrpSwitch(sigma_RpRb), spinAxis_Rp.stableNormalized(), spinRate, controlPeriod};
    }

    static bool isValidSigma_RbR0(const Eigen::Vector3f& sigma) { return sigma.allFinite(); }
    static bool isValidSigma_RpRb(const Eigen::Vector3f& sigma) { return sigma.allFinite(); }
    static bool isValidSpinAxis_Rp(const Eigen::Vector3f& spinAxis_Rp) {
        constexpr float kUnitNormTol = 1e-3F;
        return spinAxis_Rp.allFinite() && fabsf(spinAxis_Rp.stableNorm() - 1.0F) <= kUnitNormTol;
    }
    static bool isValidSpinRate(float spinRate) { return fsw::is_finite(spinRate); }
    static bool isValidControlPeriod(float period) { return fsw::is_finite(period) && period > 0.0F; }

    const Eigen::Vector3f& getSigma_RbR0() const { return this->sigma_RbR0; }
    const Eigen::Vector3f& getSigma_RpRb() const { return this->sigma_RpRb; }
    const Eigen::Vector3f& getSpinAxis_Rp() const { return this->spinAxis_Rp; }
    float getSpinRate() const { return this->spinRate; }
    float getControlPeriod() const { return this->controlPeriod; }

   private:
    // NOLINTBEGIN(bugprone-easily-swappable-parameters)
    MrpRotationConfig(const Eigen::Vector3f& sigma_RbR0,
                      const Eigen::Vector3f& sigma_RpRb,
                      const Eigen::Vector3f& spinAxis_Rp,
                      float spinRate,
                      float controlPeriod)
        : sigma_RbR0(sigma_RbR0),
          sigma_RpRb(sigma_RpRb),
          spinAxis_Rp(spinAxis_Rp),
          spinRate(spinRate),
          controlPeriod(controlPeriod) {}
    // NOLINTEND(bugprone-easily-swappable-parameters)

    Eigen::Vector3f sigma_RbR0;   //!< [-] base frame: Rb wrt input reference R0
    Eigen::Vector3f sigma_RpRb;   //!< [-] commanded pointing offset: frame Rp wrt Rb, in body axes
    Eigen::Vector3f spinAxis_Rp;  //!< [-] unit spin axis of R wrt Rp, in Rp axes
    float spinRate;               //!< [rad/s] signed spin rate of R wrt Rp about spinAxis_Rp
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

    Eigen::Matrix3f dcm_RpR0{Eigen::Matrix3f::Identity()};  //!< [-] cached [RpRb][RbR0], set by setConfig
    Eigen::Vector3f sigma_RRp{Eigen::Vector3f::Zero()};     //!< [-] integrated rotation of R wrt Rp
};

#endif
