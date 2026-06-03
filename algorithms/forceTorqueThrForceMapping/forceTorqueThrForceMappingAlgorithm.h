#ifndef F32XMERA_FORCE_TORQUE_THR_FORCE_MAPPING_ALGORITHM_H
#define F32XMERA_FORCE_TORQUE_THR_FORCE_MAPPING_ALGORITHM_H

#include "msgPayloadDef/definitions.h"
#include "utilities/freestandingInvalidArgument.h"
#include <math.h>
#include <stdint.h>
#include <Eigen/Core>
#include <array>
#include <cstdint>

/*! @brief Single thruster configuration */
struct ThrusterConfiguration {
    std::array<float, 3> rThrust_B{};     //!< [m] Location of the thruster in the spacecraft body frame
    std::array<float, 3> tHatThrust_B{};  //!< [-] Unit vector of the thrust direction
};

/*! @brief Thruster array configuration */
struct ThrusterArrayConfiguration {
    std::uint32_t numThrusters{};                                //!< [-] number of thrusters
    std::array<ThrusterConfiguration, MAX_EFF_CNT> thrusters{};  //!< [-] array of thruster configuration information
};

/*! @brief Validated configuration for the force/torque-to-thruster-force mapping algorithm.
 *
 * Construct via create(), which rejects an invalid thruster array (bad count or non-unit direction), a
 * non-finite center of mass, or an unrealizable mapping (an asserted desiredControlAxes_B axis is
 * uncontrollable, or the geometry is ill-conditioned with condition number above 100). desiredControlAxes_B
 * are per-axis controllability assertions (torque xyz then force xyz, body frame B).
 */
class ForceTorqueThrForceMappingConfig final {
   public:
    static ForceTorqueThrForceMappingConfig create(const ThrusterArrayConfiguration& thrusters,
                                                   const Eigen::Vector3f& centerOfMass_B,
                                                   const std::array<bool, 6>& desiredControlAxes_B) {
        if (!isValidThrusters(thrusters)) {
            FSW_THROW_INVALID_ARGUMENT(
                "forceTorqueThrForceMapping: numThrusters must be in [1, MAX_EFF_CNT] and each thruster "
                "direction must be a unit vector");
        }
        if (!isValidCenterOfMass_B(centerOfMass_B)) {
            FSW_THROW_INVALID_ARGUMENT("forceTorqueThrForceMapping: centerOfMass_B must be finite");
        }
        if (!isValidMapping(thrusters, centerOfMass_B, desiredControlAxes_B)) {
            FSW_THROW_INVALID_ARGUMENT(
                "forceTorqueThrForceMapping: the configuration does not yield a valid thruster mapping -- an "
                "axis marked in desiredControlAxes_B is not controllable by the thruster array, or the thruster "
                "geometry is ill-conditioned (condition number above 100).");
        }
        return {thrusters, centerOfMass_B, desiredControlAxes_B};
    }

    static bool isValidThrusters(const ThrusterArrayConfiguration& thrusters) {
        if (thrusters.numThrusters == 0 || thrusters.numThrusters > MAX_EFF_CNT) {
            return false;
        }
        constexpr float normTolerance = 1e-3F;
        for (std::uint32_t i = 0; i < thrusters.numThrusters; ++i) {
            const Eigen::Vector3f direction(thrusters.thrusters.at(i).tHatThrust_B.data());
            if (fabsf(direction.stableNorm() - 1.0F) > normTolerance) {
                return false;
            }
        }
        return true;
    }
    static bool isValidCenterOfMass_B(const Eigen::Vector3f& centerOfMass_B) { return centerOfMass_B.allFinite(); }
    // No isValidDesiredControlAxes — any bool combination is valid; controllability is checked against control mapping
    // matrix DG.

    // True if the mapping is realizable: every asserted axis is controllable and the control mapping
    // matrix DG is well-conditioned (condition number below 100).
    // In the .cpp because it shares the mapping computation with the algorithm.
    static bool isValidMapping(const ThrusterArrayConfiguration& thrusters,
                               const Eigen::Vector3f& centerOfMass_B,
                               const std::array<bool, 6>& desiredControlAxes_B);

    const ThrusterArrayConfiguration& getThrusters() const { return thrusters; }
    Eigen::Vector3f getCenterOfMass_B() const { return centerOfMass_B; }
    const std::array<bool, 6>& getDesiredControlAxes() const { return desiredControlAxes_B; }

   private:
    ForceTorqueThrForceMappingConfig(const ThrusterArrayConfiguration& thrusters,
                                     const Eigen::Vector3f& centerOfMass_B,
                                     const std::array<bool, 6>& desiredControlAxes_B)
        : thrusters(thrusters), centerOfMass_B(centerOfMass_B), desiredControlAxes_B(desiredControlAxes_B) {}

    ThrusterArrayConfiguration thrusters;
    Eigen::Vector3f centerOfMass_B;
    std::array<bool, 6> desiredControlAxes_B;
};

/*! @brief This module maps thruster forces for arbitrary forces and torques
 */
class ForceTorqueThrForceMappingAlgorithm final {
   public:
    explicit ForceTorqueThrForceMappingAlgorithm(const ForceTorqueThrForceMappingConfig& config);

    void setConfig(const ForceTorqueThrForceMappingConfig& config);

    Eigen::Vector<float, MAX_EFF_CNT> update(const Eigen::Vector3f& cmdTorque_B,
                                             const Eigen::Vector3f& cmdForce_B) const;

   private:
    ForceTorqueThrForceMappingConfig cfg;  //!< validated configuration (thrusters, CoM, controllability assertions)
    Eigen::Matrix<float, MAX_EFF_CNT, 6> pseudoInverseDG{
        Eigen::Matrix<float, MAX_EFF_CNT, 6>::Zero()};  //!< truncated-SVD pseudo-inverse of DG
};

#endif
