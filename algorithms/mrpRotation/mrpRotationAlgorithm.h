#ifndef F32XMERA_MRP_ROTATION_ALGORITHM_H
#define F32XMERA_MRP_ROTATION_ALGORITHM_H

#include "msgPayloadDef/AttRefMsgF32Payload.h"
#include "msgPayloadDef/AttStateMsgF32Payload.h"
#include <stdint.h>
#include <Eigen/Core>

/*! @brief MRP Rotation class */
class MrpRotationAlgorithm {
   public:
    void reset();
    AttRefMsgF32Payload update(uint64_t callTime, AttRefMsgF32Payload inputRef, AttStateMsgF32Payload attStates);

    void checkRasterCommands();
    void computeTimeStep(uint64_t callTime);
    AttRefMsgF32Payload computeMRPRotationReference(Eigen::Vector3f sigma_R0N,
                                                    Eigen::Vector3f omega_R0N_N,
                                                    Eigen::Vector3f domega_R0N_N);

    void setSigmaRR0(const Eigen::Vector3f& sigma);
    const Eigen::Vector3f getSigmaRR0() const;
    void setOmegaRR0(const Eigen::Vector3f& omega);
    const Eigen::Vector3f getOmegaRR0() const;
    void enableDynamicReference();
    const bool isDynamicReferenceEnabled() const;

   private:
    Eigen::Vector3f sigma_RR0{
        Eigen::Vector3f::Zero()};  //!< [-] current MRP attitude coordinate set with respect to the input reference
    Eigen::Vector3f omega_RR0_R{
        Eigen::Vector3f::Zero()};  //!< [rad/s] angular velocity vector relative to input reference
    Eigen::Vector3f cmdSet{
        Eigen::Vector3f::Zero()};  //!< [] msg commanded initial MRP sigma_RR0 set with respect to input reference
    Eigen::Vector3f cmdRates{
        Eigen::Vector3f::Zero()};  //!< [rad/s] msg commanded constant angular velocity vector omega_RR0_R
    Eigen::Vector3f priorCmdSet{Eigen::Vector3f::Zero()};    //!< [] prior commanded MRP set
    Eigen::Vector3f priorCmdRates{Eigen::Vector3f::Zero()};  //!< [rad/s] prior commanded angular velocity vector
    uint64_t priorTime{};                                    //!< [ns] last time the guidance module is called
    float dt{};                                              //!< [s] integration time-step
    bool
        dynamicReferenceEnabled{};  //!< true if desired attitude input message is linked to provide a dynamic reference
};

#endif
