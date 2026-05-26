#ifndef F32XMERA_MRP_ROTATION_ALGORITHM_H
#define F32XMERA_MRP_ROTATION_ALGORITHM_H

#include "mrpRotationTypes.h"
#include "msgPayloadDef/AttRefMsgF32Payload.h"
#include "msgPayloadDef/AttStateMsgF32Payload.h"
#include "utilities/freestandingInvalidArgument.h"
#include <stdint.h>
#include <Eigen/Core>

class MrpRotationConfig final {
   public:
    static MrpRotationConfig create(const Eigen::Vector3f& initialSigmaRR0,
                                    const Eigen::Vector3f& omegaRR0R,
                                    bool dynamicReferenceEnabled) {
        if (!isValidInitialSigmaRR0(initialSigmaRR0)) {
            FSW_THROW_INVALID_ARGUMENT("mrpRotation: initialSigmaRR0 must be finite");
        }
        if (!isValidOmegaRR0R(omegaRR0R)) {
            FSW_THROW_INVALID_ARGUMENT("mrpRotation: omegaRR0R must be finite");
        }
        return {initialSigmaRR0, omegaRR0R, dynamicReferenceEnabled};
    }

    static bool isValidInitialSigmaRR0(const Eigen::Vector3f& sigma) { return sigma.allFinite(); }
    static bool isValidOmegaRR0R(const Eigen::Vector3f& omega) { return omega.allFinite(); }

    Eigen::Vector3f getInitialSigmaRR0() const { return initialSigmaRR0; }
    Eigen::Vector3f getOmegaRR0R() const { return omegaRR0R; }
    bool getDynamicReferenceEnabled() const { return dynamicReferenceEnabled; }

   private:
    MrpRotationConfig(const Eigen::Vector3f& initialSigmaRR0,
                      const Eigen::Vector3f& omegaRR0R,
                      bool dynamicReferenceEnabled)
        : initialSigmaRR0(initialSigmaRR0), omegaRR0R(omegaRR0R), dynamicReferenceEnabled(dynamicReferenceEnabled) {}

    Eigen::Vector3f initialSigmaRR0;
    Eigen::Vector3f omegaRR0R;
    bool dynamicReferenceEnabled;
};

/*! @brief MRP Rotation algorithm class. */
class MrpRotationAlgorithm final {
   public:
    explicit MrpRotationAlgorithm(const MrpRotationConfig& config);

    void setConfig(const MrpRotationConfig& config);

    void reset();
    AttRefMsgF32Payload update(uint64_t callTime, AttRefMsgF32Payload inputRef, AttStateMsgF32Payload attStates);

   private:
    void checkRasterCommands();
    void computeTimeStep(uint64_t callTime);
    AttRefMsgF32Payload computeMRPRotationReference(const Eigen::Vector3f& sigma_R0N,
                                                    const Eigen::Vector3f& omega_R0N_N,
                                                    const Eigen::Vector3f& domega_R0N_N);

    MrpRotationConfig cfg;

    Eigen::Vector3f sigma_RR0{Eigen::Vector3f::Zero()};      //!< [-] integrated MRP attitude relative to input ref
    Eigen::Vector3f omega_RR0_R{Eigen::Vector3f::Zero()};    //!< [rad/s] active angular velocity relative to input ref
    Eigen::Vector3f cmdSet{Eigen::Vector3f::Zero()};         //!< [-] last commanded MRP set
    Eigen::Vector3f cmdRates{Eigen::Vector3f::Zero()};       //!< [rad/s] last commanded angular velocity
    Eigen::Vector3f priorCmdSet{Eigen::Vector3f::Zero()};    //!< [-] prior commanded MRP set (for change detection)
    Eigen::Vector3f priorCmdRates{Eigen::Vector3f::Zero()};  //!< [rad/s] prior commanded angular velocity
    uint64_t priorTime{};                                    //!< [ns] last time the guidance module was called
    float dt{};                                              //!< [s] integration time step
};

#endif
