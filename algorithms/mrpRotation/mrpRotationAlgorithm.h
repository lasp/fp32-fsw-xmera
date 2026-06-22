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
    static MrpRotationConfig create(const Eigen::Vector3f& initialSigmaRR0,
                                    const Eigen::Vector3f& omegaRR0R,
                                    float controlPeriod) {
        if (!isValidInitialSigmaRR0(initialSigmaRR0)) {
            FSW_THROW_INVALID_ARGUMENT("mrpRotation: initialSigmaRR0 must be finite");
        }
        if (!isValidOmegaRR0R(omegaRR0R)) {
            FSW_THROW_INVALID_ARGUMENT("mrpRotation: omegaRR0R must be finite");
        }
        if (!isValidControlPeriod(controlPeriod)) {
            FSW_THROW_INVALID_ARGUMENT("mrpRotation: controlPeriod must be > 0");
        }
        // Bound the seed MRP to the principal set (norm <= 1) by switching to the shadow set if
        // needed, so the stored initial attitude is always a well-conditioned MRP representation.
        return {mrpSwitch(initialSigmaRR0, 1.0F), omegaRR0R, controlPeriod};
    }

    static bool isValidInitialSigmaRR0(const Eigen::Vector3f& sigma) { return sigma.allFinite(); }
    static bool isValidOmegaRR0R(const Eigen::Vector3f& omega) { return omega.allFinite(); }
    static bool isValidControlPeriod(float period) { return fsw::is_finite(period) && period > 0.0F; }

    Eigen::Vector3f getInitialSigmaRR0() const { return initialSigmaRR0; }
    Eigen::Vector3f getOmegaRR0R() const { return omegaRR0R; }
    float getControlPeriod() const { return controlPeriod; }

   private:
    MrpRotationConfig(const Eigen::Vector3f& initialSigmaRR0, const Eigen::Vector3f& omegaRR0R, float controlPeriod)
        : initialSigmaRR0(initialSigmaRR0), omegaRR0R(omegaRR0R), controlPeriod(controlPeriod) {}

    Eigen::Vector3f initialSigmaRR0;
    Eigen::Vector3f omegaRR0R;
    float controlPeriod;
};

/*! @brief MRP Rotation algorithm class. */
class MrpRotationAlgorithm final {
   public:
    explicit MrpRotationAlgorithm(const MrpRotationConfig& config);

    void setConfig(const MrpRotationConfig& config);

    MrpRotationOutput update(const MrpRotationAttRefInputs& attRef);

   private:
    MrpRotationConfig cfg;

    Eigen::Vector3f sigma_RR0{Eigen::Vector3f::Zero()};    //!< [-] integrated MRP attitude relative to input ref
    Eigen::Vector3f omega_RR0_R{Eigen::Vector3f::Zero()};  //!< [rad/s] active angular velocity relative to input ref
};

#endif
