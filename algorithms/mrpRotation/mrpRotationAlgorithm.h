#ifndef F32XMERA_MRP_ROTATION_ALGORITHM_H
#define F32XMERA_MRP_ROTATION_ALGORITHM_H

#include "utilities/freestandingInvalidArgument.h"
#include "utilities/freestandingIsFinite.hpp"
#include <stdint.h>
#include <Eigen/Core>

/*! @brief Algorithm-native input bundle that mirrors AttRefMsgF32Payload. The adapter converts the
 *  payload's float[3] arrays into Eigen vectors via eigenSupport.h before calling update().
 */
struct MrpRotationAttRefInputs {
    Eigen::Vector3f sigma_R0N{Eigen::Vector3f::Zero()};     //!< [-] input ref MRP attitude wrt inertial N
    Eigen::Vector3f omega_R0N_N{Eigen::Vector3f::Zero()};   //!< [rad/s] input ref angular velocity, N-frame components
    Eigen::Vector3f domega_R0N_N{Eigen::Vector3f::Zero()};  //!< [rad/s^2] input ref angular acceleration, N-frame
};

/*! @brief Algorithm-native input bundle that mirrors AttStateMsgF32Payload. Only consumed when the
 *  Config's dynamicReferenceEnabled flag is true; otherwise its contents are ignored.
 */
struct MrpRotationAttStateInputs {
    Eigen::Vector3f cmdSigma{Eigen::Vector3f::Zero()};  //!< [-] desired attitude command MRP
    Eigen::Vector3f cmdOmega{Eigen::Vector3f::Zero()};  //!< [rad/s] desired attitude command rate
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
                                    float controlPeriod,
                                    bool dynamicReferenceEnabled) {
        if (!isValidInitialSigmaRR0(initialSigmaRR0)) {
            FSW_THROW_INVALID_ARGUMENT("mrpRotation: initialSigmaRR0 must be finite");
        }
        if (!isValidOmegaRR0R(omegaRR0R)) {
            FSW_THROW_INVALID_ARGUMENT("mrpRotation: omegaRR0R must be finite");
        }
        if (!isValidControlPeriod(controlPeriod)) {
            FSW_THROW_INVALID_ARGUMENT("mrpRotation: controlPeriod must be > 0");
        }
        return {initialSigmaRR0, omegaRR0R, controlPeriod, dynamicReferenceEnabled};
    }

    static bool isValidInitialSigmaRR0(const Eigen::Vector3f& sigma) { return sigma.allFinite(); }
    static bool isValidOmegaRR0R(const Eigen::Vector3f& omega) { return omega.allFinite(); }
    static bool isValidControlPeriod(float period) { return fsw::is_finite(period) && period > 0.0F; }

    Eigen::Vector3f getInitialSigmaRR0() const { return initialSigmaRR0; }
    Eigen::Vector3f getOmegaRR0R() const { return omegaRR0R; }
    float getControlPeriod() const { return controlPeriod; }
    bool getDynamicReferenceEnabled() const { return dynamicReferenceEnabled; }

   private:
    MrpRotationConfig(const Eigen::Vector3f& initialSigmaRR0,
                      const Eigen::Vector3f& omegaRR0R,
                      float controlPeriod,
                      bool dynamicReferenceEnabled)
        : initialSigmaRR0(initialSigmaRR0),
          omegaRR0R(omegaRR0R),
          controlPeriod(controlPeriod),
          dynamicReferenceEnabled(dynamicReferenceEnabled) {}

    Eigen::Vector3f initialSigmaRR0;
    Eigen::Vector3f omegaRR0R;
    float controlPeriod;
    bool dynamicReferenceEnabled;
};

/*! @brief MRP Rotation algorithm class. */
class MrpRotationAlgorithm final {
   public:
    explicit MrpRotationAlgorithm(const MrpRotationConfig& config);

    void setConfig(const MrpRotationConfig& config);

    void reset();
    MrpRotationOutput update(const MrpRotationAttRefInputs& attRef, const MrpRotationAttStateInputs& attState);

   private:
    void checkRasterCommands();
    MrpRotationOutput computeMRPRotationReference(const Eigen::Vector3f& sigma_R0N,
                                                  const Eigen::Vector3f& omega_R0N_N,
                                                  const Eigen::Vector3f& domega_R0N_N);

    MrpRotationConfig cfg;

    Eigen::Vector3f sigma_RR0{Eigen::Vector3f::Zero()};      //!< [-] integrated MRP attitude relative to input ref
    Eigen::Vector3f omega_RR0_R{Eigen::Vector3f::Zero()};    //!< [rad/s] active angular velocity relative to input ref
    Eigen::Vector3f cmdSet{Eigen::Vector3f::Zero()};         //!< [-] last commanded MRP set
    Eigen::Vector3f cmdRates{Eigen::Vector3f::Zero()};       //!< [rad/s] last commanded angular velocity
    Eigen::Vector3f priorCmdSet{Eigen::Vector3f::Zero()};    //!< [-] prior commanded MRP set (for change detection)
    Eigen::Vector3f priorCmdRates{Eigen::Vector3f::Zero()};  //!< [rad/s] prior commanded angular velocity
};

#endif
