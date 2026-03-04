#ifndef F32XMERA_MRP_STEERING_ALGORITHM_H
#define F32XMERA_MRP_STEERING_ALGORITHM_H

#include "../msgPayloadDef/definitions.h"
#include "fswAlgorithms/fswUtilities/fswDefinitions.h"

#include <stdint.h>
#include <Eigen/Core>

/*! Struct containing the reaction wheel inputs needed by the algorithm. */
struct InputRwData {
    Eigen::Matrix<float, 3, RW_EFF_CNT> GsMatrix_B = Eigen::Matrix<float, 3, RW_EFF_CNT>::Zero();
    std::array<float, RW_EFF_CNT> JsList{};
    uint32_t numRW{};
};

/*! Struct containing the guidance inputs needed by the algorithm. */
struct InputGuidanceData {
    Eigen::Vector3f sigma_BR = Eigen::Vector3f::Zero();
    Eigen::Vector3f omega_BR_B = Eigen::Vector3f::Zero();
    Eigen::Vector3f omega_RN_B = Eigen::Vector3f::Zero();
    Eigen::Vector3f domega_RN_B = Eigen::Vector3f::Zero();
};

/*! @brief Data structure for the MRP feedback attitude control routine. */
class MrpSteeringAlgorithm final {
   public:
    void reset(const InputRwData& rwInput, bool rwIsConfigured);
    Eigen::Vector3f update(const InputGuidanceData& attGuidInput,
                           const std::array<float, RW_EFF_CNT>& wheelSpeeds,
                           const std::array<FSWdeviceAvailability, RW_EFF_CNT>& wheelAvailability);

    void setSpacecraftInertia(const Eigen::Matrix3f& inertia);
    Eigen::Matrix3f getSpacecraftInertia() const;
    void setK1(float gain);
    float getK1() const;
    void setK3(float gain);
    float getK3() const;
    void setOmegaMax(float omega);
    float getOmegaMax() const;
    void setIgnoreFeedforward(bool ignore);
    bool getIgnoreFeedforward() const;
    void setP(float gain);
    float getP() const;
    void setKi(float gain);
    float getKi() const;
    void setIntegralLimit(float limit);
    float getIntegralLimit() const;
    void setKnownTorquePntB_B(const Eigen::Vector3f& torque);
    Eigen::Vector3f getKnownTorquePntB_B() const;
    void setControlPeriod(float period);
    float getControlPeriod() const;

   private:
    float K1{};                         //!< [rad/sec] Proportional gain applied to MRP errors
    float K3{};                         //!< [rad/sec] Cubic gain applied to MRP error in steering saturation function
    float omegaMax{};                   //!< [rad/sec] Maximum rate command of steering control
    bool ignoreOuterLoopFeedforward{};  //!< [] Boolean flag indicating if outer feedforward term should be included
    float P{};                          //!< [N*m*s]   Rate error feedback gain applied
    float Ki{};                         //!< [N*m]     Integration feedback error on rate error
    float integralLimit{};              //!< [N*m]     Integration limit to avoid wind-up issue
    Eigen::Vector3f knownTorquePntB_B{
        Eigen::Vector3f::Zero()};  //!< [N*m]     known external torque in body frame vector components
    float controlPeriod{};         //!< [s] time between two algorithm update calls
    Eigen::Vector3f z{};           //!< [rad]     integral state of delta_omega
    Eigen::Matrix3f ISCPntB_B{};   //!< [kg m^2] Spacecraft Inertia
    InputRwData rwConfigParams{};  //!< [-] struct containing the reaction wheel inputs needed by the algorithm
    bool rwIsConfigured{};         //!< [-] indicates whether reaction wheels are configured through the rwConfigMsg
};

#endif
