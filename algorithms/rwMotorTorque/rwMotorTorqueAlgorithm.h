#ifndef F32XMERA_RW_MOTOR_TORQUE_ALGORITHM_H
#define F32XMERA_RW_MOTOR_TORQUE_ALGORITHM_H

#include "msgPayloadDef/definitions.h"
#include <fswAlgorithms/fswUtilities/fswDefinitions.h>

#include <Eigen/Core>
#include <array>
#include <cstdint>

inline constexpr uint32_t kMaxNumRw = RW_EFF_CNT;

/*! @brief Reaction-wheel spin-axis configuration in body-frame components. */
struct RwMotorTorqueArrayConfig {
    uint32_t numRW{};  //!< [-] number of reaction wheels on the vehicle
    Eigen::Matrix<float, 3, kMaxNumRw> GsMatrix_B{
        Eigen::Matrix<float, 3, kMaxNumRw>::Zero()};  //!< [-] RW spin axes in body frame, one column per wheel
};

/*! @brief Per-wheel reaction-wheel availability. */
struct RwMotorTorqueAvailability {
    std::array<FSWdeviceAvailability, kMaxNumRw>
        wheelAvailability{};  //!< [-] AVAILABLE / UNAVAILABLE state of each reaction wheel
};

/*! @brief Top level structure for the sub-module routines. */
class RwMotorTorqueAlgorithm {
   public:
    void configure(const RwMotorTorqueArrayConfig& rwConfig,
                   const RwMotorTorqueAvailability& availability,
                   bool rwAvailIsLinked);
    Eigen::Vector<float, kMaxNumRw> update(const Eigen::Vector3f& Lr_B) const;  //!< [N-m] RW motor torques

    void setControlAxes(const Eigen::Matrix3f& controlMappingMatrix);
    Eigen::Matrix3f getControlAxes() const;

   private:
    Eigen::Matrix3f controlAxes_B{Eigen::Matrix3f::Zero()};  //!< [-] array of the control unit axes
    uint32_t numControlAxes{};  //!< [-] counter indicating how many orthogonal axes are controlled
    uint32_t numAvailRW{};      //!< [-] number of reaction wheels available
    uint32_t numRW{};           //!< [-] number of reaction wheels on the vehicle
    Eigen::Matrix<float, 3, kMaxNumRw> CGs{
        Eigen::Matrix<float, 3, kMaxNumRw>::Zero()};                    //!< [-] The control mapping matrix [CB][G_s]
    std::array<FSWdeviceAvailability, kMaxNumRw> wheelsAvailability{};  //!< [-] Reaction wheel availability
};

#endif
