#ifndef F32XMERA_MRP_STEERING_ALGORITHM_H
#define F32XMERA_MRP_STEERING_ALGORITHM_H

#include "../msgPayloadDef/definitions.h"
#include "fswAlgorithms/fswUtilities/fswDefinitions.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/freestandingIsFinite.hpp"
#include "utilities/fsw/validInertiaCheck.h"

#include <stdint.h>
#include <Eigen/Core>
#include <algorithm>
#include <array>

inline constexpr uint32_t kMaxNumRw = RW_EFF_CNT;

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

/*! Steering-law control gains and the feedforward toggle. */
struct MrpSteeringControlParameters {
    float K1{};                         //!< [rad/s] proportional gain applied to MRP errors
    float K3{};                         //!< [rad/s] cubic gain applied to MRP error in steering saturation function
    float omegaMax{};                   //!< [rad/s] maximum rate command of steering control
    bool ignoreOuterLoopFeedforward{};  //!< [-] whether the outer-loop feedforward term is excluded
    float P{};                          //!< [N*m*s] rate error feedback gain
    float Ki{};                         //!< [N*m] integral feedback gain on the rate error
    float integralLimit{};              //!< [N*m] integral limit to avoid wind-up
    float controlPeriod{};              //!< [s] time between two algorithm update calls
};

/*!
 * @brief Validated configuration for the MRP steering control algorithm.
 *
 * An instance can only exist with finite, non-negative control gains (K1, K3, P, Ki, integralLimit), a finite
 * and positive maximum rate and control period, a finite known external torque, a valid spacecraft inertia
 * matrix, and a reaction-wheel configuration whose wheel count does not exceed the compile-time maximum with
 * finite spin-axis and wheel-inertia entries. Construct via MrpSteeringConfig::create(...).
 */
class MrpSteeringConfig final {
   public:
    static MrpSteeringConfig create(const MrpSteeringControlParameters& controlParameters,
                                    const Eigen::Vector3f& knownTorquePntB_B,
                                    const Eigen::Matrix3f& ISCPntB_B,
                                    const InputRwData& rwConfiguration,
                                    bool rwIsConfigured) {
        if (!isValidControlParameters(controlParameters)) {
            FSW_THROW_INVALID_ARGUMENT(
                "mrpSteering: control gains K1, K3, P, Ki, integralLimit must be finite and non-negative, and "
                "omegaMax and controlPeriod must be finite and positive.");
        }
        if (!isValidKnownTorque(knownTorquePntB_B)) {
            FSW_THROW_INVALID_ARGUMENT("mrpSteering: knownTorquePntB_B must be finite.");
        }
        if (!isValidInertia(ISCPntB_B)) {
            FSW_THROW_INVALID_ARGUMENT("mrpSteering: ISCPntB_B must be a valid inertia matrix.");
        }
        if (!isValidRwConfiguration(rwConfiguration)) {
            FSW_THROW_INVALID_ARGUMENT(
                "mrpSteering: rwConfiguration.numRW must not exceed the compile-time maximum, and the spin-axis "
                "matrix and wheel inertias must be finite.");
        }
        return {controlParameters, knownTorquePntB_B, ISCPntB_B, rwConfiguration, rwIsConfigured};
    }

    static bool isValidControlParameters(const MrpSteeringControlParameters& controlParameters) {
        const bool gainsValid = fsw::is_finite(controlParameters.K1) && controlParameters.K1 >= 0.0F &&
                                fsw::is_finite(controlParameters.K3) && controlParameters.K3 >= 0.0F &&
                                fsw::is_finite(controlParameters.P) && controlParameters.P >= 0.0F &&
                                fsw::is_finite(controlParameters.Ki) && controlParameters.Ki >= 0.0F &&
                                fsw::is_finite(controlParameters.integralLimit) &&
                                controlParameters.integralLimit >= 0.0F;
        const bool ratesValid = fsw::is_finite(controlParameters.omegaMax) && controlParameters.omegaMax > 0.0F &&
                                fsw::is_finite(controlParameters.controlPeriod) &&
                                controlParameters.controlPeriod > 0.0F;
        return gainsValid && ratesValid;
    }
    // No isValidIgnoreOuterLoopFeedforward -- any bool is valid.

    static bool isValidKnownTorque(const Eigen::Vector3f& knownTorquePntB_B) { return knownTorquePntB_B.allFinite(); }

    static bool isValidInertia(const Eigen::Matrix3f& ISCPntB_B) { return inertiaIsValid(ISCPntB_B); }

    static bool isValidRwConfiguration(const InputRwData& rwConfiguration) {
        if (rwConfiguration.numRW > kMaxNumRw || !rwConfiguration.GsMatrix_B.allFinite()) {
            return false;
        }
        return std::ranges::all_of(rwConfiguration.JsList, [](float Js) { return fsw::is_finite(Js); });
    }
    // No isValidRwIsConfigured -- any bool is valid.

    const MrpSteeringControlParameters& getControlParameters() const { return this->controlParameters; }
    const Eigen::Vector3f& getKnownTorquePntB_B() const { return this->knownTorquePntB_B; }
    const Eigen::Matrix3f& getSpacecraftInertia() const { return this->ISCPntB_B; }
    const InputRwData& getRwConfiguration() const { return this->rwConfiguration; }
    bool getRwIsConfigured() const { return this->rwIsConfigured; }

   private:
    MrpSteeringConfig(const MrpSteeringControlParameters& controlParameters,
                      const Eigen::Vector3f& knownTorquePntB_B,
                      const Eigen::Matrix3f& ISCPntB_B,
                      const InputRwData& rwConfiguration,
                      bool rwIsConfigured)
        : controlParameters(controlParameters),
          knownTorquePntB_B(knownTorquePntB_B),
          ISCPntB_B(ISCPntB_B),
          rwConfiguration(rwConfiguration),
          rwIsConfigured(rwIsConfigured) {}

    MrpSteeringControlParameters controlParameters;
    Eigen::Vector3f knownTorquePntB_B;
    Eigen::Matrix3f ISCPntB_B;
    InputRwData rwConfiguration;
    bool rwIsConfigured;
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
