#ifndef F32XMERA_DV_MANEUVER_ALGORITHM_H
#define F32XMERA_DV_MANEUVER_ALGORITHM_H

#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/freestandingIsFinite.hpp"
#include <stdint.h>
#include <Eigen/Core>

/// Burn state machine state. Complete is terminal until reInitialize().
enum class DvManeuverBurnState : uint8_t { Pending = 0, Executing = 1, Complete = 2 };

/// Burn state and the body force command produced each update.
struct DvManeuverOutput {
    DvManeuverBurnState state = DvManeuverBurnState::Pending;  ///< [-] burn state after this update
    Eigen::Vector3f cmdForce_B = Eigen::Vector3f::Zero();      ///< [N] configured force while executing, else zero
};

/// Validated, immutable configuration for the delta-V burn executor. Construct via create(), which
/// enforces the parameter constraints and throws fsw::invalid_argument on a violation.
class DvManeuverConfig final {
   public:
    // minTime, maxTime, and controlPeriod share the float type, and cmdForce_B and cmdDv_N share the Eigen::Vector3f
    // type, but each has a distinct role; construction is funneled through the named create() factory, which makes
    // the argument roles explicit at every call site.
    // NOLINTBEGIN(bugprone-easily-swappable-parameters)
    static DvManeuverConfig create(float minTime,
                                   float maxTime,
                                   float controlPeriod,
                                   const Eigen::Vector3f& cmdForce_B,
                                   const Eigen::Vector3f& cmdDv_N,
                                   uint64_t burnStartTime) {
        if (!isValidMinTime(minTime)) {
            FSW_THROW_INVALID_ARGUMENT("dvManeuver: minTime must be non-negative and finite.");
        }
        if (!isValidMaxTime(maxTime)) {
            FSW_THROW_INVALID_ARGUMENT("dvManeuver: maxTime must be positive and finite.");
        }
        if (!isValidControlPeriod(controlPeriod)) {
            FSW_THROW_INVALID_ARGUMENT("dvManeuver: controlPeriod must be positive and finite.");
        }
        if (!isValidMaxTimeRelativeToMinTime(minTime, maxTime)) {
            FSW_THROW_INVALID_ARGUMENT("dvManeuver: maxTime must be greater than minTime.");
        }
        if (!isValidCmdForce(cmdForce_B)) {
            FSW_THROW_INVALID_ARGUMENT("dvManeuver: cmdForce_B must be finite.");
        }
        if (!isValidCmdDv(cmdDv_N)) {
            FSW_THROW_INVALID_ARGUMENT("dvManeuver: cmdDv_N must be finite.");
        }
        return {minTime, maxTime, controlPeriod, cmdForce_B, cmdDv_N, burnStartTime};
    }

    static bool isValidMinTime(float minTime) { return minTime >= 0.0F && fsw::is_finite(minTime); }
    static bool isValidMaxTime(float maxTime) { return maxTime > 0.0F && fsw::is_finite(maxTime); }
    static bool isValidMaxTimeRelativeToMinTime(float minTime, float maxTime) { return maxTime > minTime; }
    static bool isValidControlPeriod(float controlPeriod) {
        return controlPeriod > 0.0F && fsw::is_finite(controlPeriod);
    }
    static bool isValidCmdForce(const Eigen::Vector3f& cmdForce_B) { return cmdForce_B.allFinite(); }
    static bool isValidCmdDv(const Eigen::Vector3f& cmdDv_N) { return cmdDv_N.allFinite(); }

    float getMinTime() const { return minTime; }
    float getMaxTime() const { return maxTime; }
    float getControlPeriod() const { return controlPeriod; }
    const Eigen::Vector3f& getCmdForce() const { return cmdForce_B; }
    const Eigen::Vector3f& getCmdDv() const { return cmdDv_N; }
    uint64_t getBurnStartTime() const { return burnStartTime; }

   private:
    DvManeuverConfig(float minTime,
                     float maxTime,
                     float controlPeriod,
                     const Eigen::Vector3f& cmdForce_B,
                     const Eigen::Vector3f& cmdDv_N,
                     uint64_t burnStartTime)
        : minTime(minTime),
          maxTime(maxTime),
          controlPeriod(controlPeriod),
          cmdForce_B(cmdForce_B),
          cmdDv_N(cmdDv_N),
          burnStartTime(burnStartTime) {}
    // NOLINTEND(bugprone-easily-swappable-parameters)

    float minTime;
    float maxTime;
    float controlPeriod;
    Eigen::Vector3f cmdForce_B;
    Eigen::Vector3f cmdDv_N;
    uint64_t burnStartTime;
};

/// Executes a delta-V burn: compares the accumulated delta-V against the commanded delta-V and,
/// subject to minimum/maximum burn-time gates, decides when the burn is complete. While the burn
/// executes it commands the configured body force, otherwise a zero force. The module holds its own
/// burn state machine across updates.
class DvManeuverAlgorithm final {
   public:
    explicit DvManeuverAlgorithm(const DvManeuverConfig& config);

    /// Installs the configuration parameters. Does not touch runtime state.
    void setConfig(const DvManeuverConfig& config);

    /// Resets the burn state machine to its initial (pre-burn) condition.
    void reInitialize();

    /// Advances the burn state machine one step.
    /// @param callTime       Evaluation time [ns].
    /// @param dvAccumulated  Total accumulated delta-V from navigation [m/s].
    /// @return Burn state and body force command for this step.
    DvManeuverOutput update(uint64_t callTime, const Eigen::Vector3f& dvAccumulated);

   private:
    DvManeuverConfig cfg;
    Eigen::Vector3f dvInitial = Eigen::Vector3f::Zero();       ///< [m/s] accumulated delta-V latched at burn start
    DvManeuverBurnState state = DvManeuverBurnState::Pending;  ///< [-] burn state machine state
    float burnTime{};                                          ///< [s] elapsed burn time
};

#endif
