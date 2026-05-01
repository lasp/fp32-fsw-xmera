#ifndef F32XMERA_RW_MOTOR_TORQUE_ALGORITHM_H
#define F32XMERA_RW_MOTOR_TORQUE_ALGORITHM_H

#include <stdint.h>

#include "msgPayloadDef/CmdTorqueBodyMsgF32Payload.h"
#include "msgPayloadDef/RWArrayConfigMsgF32Payload.h"
#include "msgPayloadDef/RwMotorTorqueMsgF32Payload.h"
#include <architecture/msgPayloadDef/RWAvailabilityMsgPayload.h>

#include <Eigen/Core>

/*! @brief Top level structure for the sub-module routines. */
class RwMotorTorqueAlgorithm {
   public:
    void configure(RWArrayConfigMsgF32Payload& rwParamsInMsg,
                   RWAvailabilityMsgPayload& wheelsAvailability,
                   bool rwAvailIsLinked);
    RwMotorTorqueMsgF32Payload update(CmdTorqueBodyMsgF32Payload& LrInputMsg,
                                      CmdTorqueBodyMsgF32Payload& LrInput2Msg,
                                      bool cmdTorque2IsLinked);

    void setControlAxes(const Eigen::Matrix3f& controlMappingMatrix);
    Eigen::Matrix3f getControlAxes() const;

   private:
    Eigen::Matrix3f controlAxes_B{Eigen::Matrix3f::Zero()};  //!< [-] array of the control unit axes
    uint32_t numControlAxes{};  //!< [-] counter indicating how many orthogonal axes are controlled
    uint32_t numAvailRW{};      //!< [-] number of reaction wheels available
    RWArrayConfigMsgF32Payload
        rwConfigParams{};  //!< [-] struct to store message containing RW config parameters in body B frame
    Eigen::Matrix<float, 3, RW_EFF_CNT> CGs{
        Eigen::Matrix<float, 3, RW_EFF_CNT>::Zero()};                    //!< [-] The control mapping matrix [CB][G_s]
    std::array<FSWdeviceAvailability, RW_EFF_CNT> wheelsAvailability{};  //!< [-] Reaction wheel availability
};

#endif
