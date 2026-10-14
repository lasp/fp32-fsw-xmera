#ifndef F32XMERA_DV_MANEUVER_ALGORITHM_H
#define F32XMERA_DV_MANEUVER_ALGORITHM_H

#include "utilities/fsw/freestandingInvalidArgument.h"
#include <stdint.h>
#include <Eigen/Core>

/// Burn state machine state. Complete is terminal until reInitialize().
enum class DvManeuverBurnState : uint8_t { Executing = 0, Complete = 1 };

/// Burn state and the body force command produced each update.
struct DvManeuverOutput {
    DvManeuverBurnState state = DvManeuverBurnState::Executing;  ///< [-] burn state after this update
    Eigen::Vector3f cmdForce_B = Eigen::Vector3f::Zero();        ///< [N] configured force while executing, else zero
};

/// Validated, immutable configuration for the delta-V burn executor. Construct via create(), which
/// enforces the parameter constraints and throws fsw::invalid_argument on a violation.
class DvManeuverConfig final {
   public:
    // minTime, maxTime, and controlPeriod share the uint64_t type, and cmdForce_B and cmdDv_N share the
    // Eigen::Vector3f type, but each has a distinct role; construction is funneled through the named create()
    // factory, which makes the argument roles explicit at every call site.
    // NOLINTBEGIN(bugprone-easily-swappable-parameters)
    static DvManeuverConfig create(uint64_t minTime,
                                   uint64_t maxTime,
                                   uint64_t controlPeriod,
                                   const Eigen::Vector3f& cmdForce_B,
                                   const Eigen::Vector3f& cmdDv_N) {
        if (!isValidMaxTime(maxTime)) {
            FSW_THROW_INVALID_ARGUMENT("dvManeuver: maxTime must be positive.");
        }
        if (!isValidMaxTimeRelativeToMinTime(minTime, maxTime)) {
            FSW_THROW_INVALID_ARGUMENT("dvManeuver: maxTime must be greater than minTime.");
        }
        if (!isValidControlPeriod(controlPeriod)) {
            FSW_THROW_INVALID_ARGUMENT("dvManeuver: controlPeriod must be positive.");
        }
        if (!isValidCmdForce(cmdForce_B)) {
            FSW_THROW_INVALID_ARGUMENT("dvManeuver: cmdForce_B must be finite.");
        }
        if (!isValidCmdDv(cmdDv_N)) {
            FSW_THROW_INVALID_ARGUMENT("dvManeuver: cmdDv_N must be finite.");
        }
        return {minTime, maxTime, controlPeriod, cmdForce_B, cmdDv_N};
    }

    static bool isValidMaxTime(uint64_t maxTime) { return maxTime > 0U; }
    static bool isValidMaxTimeRelativeToMinTime(uint64_t minTime, uint64_t maxTime) { return maxTime > minTime; }
    static bool isValidControlPeriod(uint64_t controlPeriod) { return controlPeriod > 0U; }
    static bool isValidCmdForce(const Eigen::Vector3f& cmdForce_B) { return cmdForce_B.allFinite(); }
    static bool isValidCmdDv(const Eigen::Vector3f& cmdDv_N) { return cmdDv_N.allFinite(); }

    uint64_t getMinTime() const { return minTime; }
    uint64_t getMaxTime() const { return maxTime; }
    uint64_t getControlPeriod() const { return controlPeriod; }
    const Eigen::Vector3f& getCmdForce() const { return cmdForce_B; }
    const Eigen::Vector3f& getCmdDv() const { return cmdDv_N; }

   private:
    DvManeuverConfig(uint64_t minTime,
                     uint64_t maxTime,
                     uint64_t controlPeriod,
                     const Eigen::Vector3f& cmdForce_B,
                     const Eigen::Vector3f& cmdDv_N)
        : minTime(minTime), maxTime(maxTime), controlPeriod(controlPeriod), cmdForce_B(cmdForce_B), cmdDv_N(cmdDv_N) {}
    // NOLINTEND(bugprone-easily-swappable-parameters)

    uint64_t minTime;
    uint64_t maxTime;
    uint64_t controlPeriod;
    Eigen::Vector3f cmdForce_B;
    Eigen::Vector3f cmdDv_N;
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
    /// @param dvAccumulated  Total accumulated delta-V from navigation [m/s].
    /// @return Burn state and body force command for this step.
    DvManeuverOutput update(const Eigen::Vector3f& dvAccumulated);

   private:
    DvManeuverConfig cfg;
    float cmdDvMagnitude{};  ///< [m/s] magnitude of the configured cmdDv_N, cached by setConfig()
    DvManeuverBurnState state = DvManeuverBurnState::Executing;  ///< [-] burn state machine state
    uint64_t burnTime{};  ///< [ns] burn time at this update: controlPeriod per executing update so far
};

#endif
