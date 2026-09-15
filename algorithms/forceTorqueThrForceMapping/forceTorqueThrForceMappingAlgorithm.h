#ifndef F32XMERA_FORCE_TORQUE_THR_FORCE_MAPPING_ALGORITHM_H
#define F32XMERA_FORCE_TORQUE_THR_FORCE_MAPPING_ALGORITHM_H

#include "forceTorqueThrForceMappingTypes.h"

#include "msgPayloadDef/definitions.h"
#include "utilities/fsw/freestandingInvalidArgument.h"

#include <Eigen/Core>

#include <math.h>
#include <array>

/*! @brief Single thruster configuration */
struct ThrusterConfiguration {
    std::array<float, 3> r_TB_B{};  //!< [m] Location of the thruster in the spacecraft body frame
    std::array<float, 3> tHat_B{};  //!< [-] Unit vector of the thrust direction
};

/*! @brief Thruster array configuration */
struct ThrusterArrayConfiguration {
    std::uint32_t numThrusters{};  //!< [-] number of thrusters
    std::array<ThrusterConfiguration, kMaxThrusterCount>
        thrusters{};  //!< [-] array of thruster configuration information
};

/*! @brief Validated configuration for the force/torque-to-thruster-force mapping algorithm.
 *
 * Construct via create(), which rejects an invalid thruster array (bad count or non-unit direction), a
 * non-finite center of mass, an empty axis selection, or an unrealizable mapping (a desiredControlAxes_B
 * axis is uncontrollable, or the geometry is ill-conditioned with condition number above 100).
 *
 * desiredControlAxes_B selects the axes that the mapping controls (torque xyz then force xyz, body frame B).
 * Only the selected rows of the control mapping matrix DG enter the solve. The solve thus applies no
 * condition to an unselected axis, and does not balance such an axis against the selected ones. The
 * selection must contain a minimum of one axis.
 */
class ForceTorqueThrForceMappingConfig final {
   public:
    static ForceTorqueThrForceMappingConfig create(const ThrusterArrayConfiguration& thrusters,
                                                   const Eigen::Vector3f& centerOfMass_B,
                                                   const std::array<bool, 6>& desiredControlAxes_B) {
        if (!isValidThrusters(thrusters)) {
            FSW_THROW_INVALID_ARGUMENT(
                "forceTorqueThrForceMapping: numThrusters must be in [1, kMaxThrusterCount] and each thruster "
                "direction must be a unit vector");
        }
        if (!isValidCenterOfMass_B(centerOfMass_B)) {
            FSW_THROW_INVALID_ARGUMENT("forceTorqueThrForceMapping: centerOfMass_B must be finite");
        }
        if (!isValidDesiredControlAxes_B(desiredControlAxes_B)) {
            FSW_THROW_INVALID_ARGUMENT(
                "forceTorqueThrForceMapping: desiredControlAxes_B must select at least one control axis.");
        }
        if (!isValidMapping(thrusters, centerOfMass_B, desiredControlAxes_B)) {
            FSW_THROW_INVALID_ARGUMENT(
                "forceTorqueThrForceMapping: the configuration does not yield a valid thruster mapping -- an "
                "axis selected in desiredControlAxes_B is not controllable by the thruster array, or the thruster "
                "geometry is ill-conditioned (condition number above 100).");
        }
        return {thrusters, centerOfMass_B, desiredControlAxes_B};
    }

    static bool isValidThrusters(const ThrusterArrayConfiguration& thrusters) {
        if (thrusters.numThrusters == 0 || thrusters.numThrusters > kMaxThrusterCount) {
            return false;
        }
        constexpr float normTolerance = 1e-3F;
        for (std::uint32_t i = 0; i < thrusters.numThrusters; ++i) {
            const Eigen::Vector3f direction(thrusters.thrusters.at(i).tHat_B.data());
            if (fabsf(direction.stableNorm() - 1.0F) > normTolerance) {
                return false;
            }
        }
        return true;
    }
    static bool isValidCenterOfMass_B(const Eigen::Vector3f& centerOfMass_B) { return centerOfMass_B.allFinite(); }
    // The selection names the axes the mapping controls, so it must name at least one. Which of the selected
    // axes are reachable is a property of the geometry and is checked against DG by isValidMapping.
    static bool isValidDesiredControlAxes_B(const std::array<bool, 6>& desiredControlAxes_B) {
        return desiredControlAxes_B.at(0) || desiredControlAxes_B.at(1) || desiredControlAxes_B.at(2) ||
               desiredControlAxes_B.at(3) || desiredControlAxes_B.at(4) || desiredControlAxes_B.at(5);
    }

    // True if the mapping is realizable: every selected axis is controllable and the selected rows of the
    // control mapping matrix DG are well-conditioned (condition number below 100).
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

    Eigen::Vector<float, kMaxThrusterCount> update(const Eigen::Vector3f& cmdTorque_B,
                                                   const Eigen::Vector3f& cmdForce_B) const;

   private:
    ForceTorqueThrForceMappingConfig cfg;  //!< validated configuration (thrusters, CoM, control-axis selection)
    Eigen::Matrix<float, kMaxThrusterCount, 6> pseudoInverseDG{
        Eigen::Matrix<float, kMaxThrusterCount, 6>::Zero()};  //!< truncated-SVD pseudo-inverse of the selected DG rows
    Eigen::Vector<float, kMaxThrusterCount> nullSpaceShift{
        Eigen::Vector<float, kMaxThrusterCount>::Zero()};  //!< [-] shift direction in the null space of selected DG
};

#endif
