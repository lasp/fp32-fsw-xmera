#ifndef F32XMERA_DV_MANEUVER_ALGORITHM_H
#define F32XMERA_DV_MANEUVER_ALGORITHM_H

#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/freestandingIsFinite.hpp"
#include "utilities/fsw/timeConstants.h"
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
/// enforces the parameter constraints and throws fsw::invalid_argument on a violation. The times are given in
/// seconds and stored as nanoseconds, rounded to the nearest nanosecond.
class DvManeuverConfig final {
   public:
    /// [s] Upper bound on every time parameter, so that its nanosecond value fits in uint64_t.
    static constexpr double kMaxTime = 1.0e9;

    // minTime, maxTime, and controlPeriod share the double type, and cmdForce_B and cmdDv_N share the
    // Eigen::Vector3f type, but each has a distinct role; construction is funneled through the named create()
    // factory, which makes the argument roles explicit at every call site.
    // NOLINTBEGIN(bugprone-easily-swappable-parameters)
    static DvManeuverConfig create(double minTime,
                                   double maxTime,
                                   double controlPeriod,
                                   const Eigen::Vector3f& cmdForce_B,
                                   const Eigen::Vector3f& cmdDv_N) {
        if (!isValidMinTime(minTime)) {
            FSW_THROW_INVALID_ARGUMENT("dvManeuver: minTime must be finite and in [0, 1e9] s.");
        }
        if (!isValidMaxTime(maxTime)) {
            FSW_THROW_INVALID_ARGUMENT("dvManeuver: maxTime must be finite and in (0, 1e9] s.");
        }
        if (!isValidMaxTimeRelativeToMinTime(minTime, maxTime)) {
            FSW_THROW_INVALID_ARGUMENT("dvManeuver: maxTime must be greater than minTime.");
        }
        if (!isValidControlPeriod(controlPeriod)) {
            FSW_THROW_INVALID_ARGUMENT("dvManeuver: controlPeriod must be finite, at least 1 ns, and at most 1e9 s.");
        }
        if (!isValidCmdForce(cmdForce_B)) {
            FSW_THROW_INVALID_ARGUMENT("dvManeuver: cmdForce_B must be finite.");
        }
        if (!isValidCmdDv(cmdDv_N)) {
            FSW_THROW_INVALID_ARGUMENT("dvManeuver: cmdDv_N must be finite.");
        }
        return {toNanoseconds(minTime), toNanoseconds(maxTime), toNanoseconds(controlPeriod), cmdForce_B, cmdDv_N};
    }

    static bool isValidMinTime(double minTime) {
        return fsw::is_finite(minTime) && minTime >= 0.0 && minTime <= kMaxTime;
    }
    static bool isValidMaxTime(double maxTime) {
        return fsw::is_finite(maxTime) && maxTime > 0.0 && maxTime <= kMaxTime;
    }
    /// Compares the rounded nanosecond values, so two limits that round to the same nanosecond are rejected.
    /// Call it only with a valid minTime and maxTime.
    static bool isValidMaxTimeRelativeToMinTime(double minTime, double maxTime) {
        return toNanoseconds(maxTime) > toNanoseconds(minTime);
    }
    static bool isValidControlPeriod(double controlPeriod) {
        return fsw::is_finite(controlPeriod) && controlPeriod > 0.0 && controlPeriod <= kMaxTime &&
               toNanoseconds(controlPeriod) > 0U;
    }
    static bool isValidCmdForce(const Eigen::Vector3f& cmdForce_B) { return cmdForce_B.allFinite(); }
    static bool isValidCmdDv(const Eigen::Vector3f& cmdDv_N) { return cmdDv_N.allFinite(); }

    uint64_t getMinTimeNs() const { return minTimeNs; }
    uint64_t getMaxTimeNs() const { return maxTimeNs; }
    uint64_t getControlPeriodNs() const { return controlPeriodNs; }
    const Eigen::Vector3f& getCmdForce() const { return cmdForce_B; }
    const Eigen::Vector3f& getCmdDv() const { return cmdDv_N; }

   private:
    DvManeuverConfig(uint64_t minTimeNs,
                     uint64_t maxTimeNs,
                     uint64_t controlPeriodNs,
                     const Eigen::Vector3f& cmdForce_B,
                     const Eigen::Vector3f& cmdDv_N)
        : minTimeNs(minTimeNs),
          maxTimeNs(maxTimeNs),
          controlPeriodNs(controlPeriodNs),
          cmdForce_B(cmdForce_B),
          cmdDv_N(cmdDv_N) {}
    // NOLINTEND(bugprone-easily-swappable-parameters)

    /// Rounds to the nearest nanosecond, halves up. Requires a finite time in [0, kMaxTime]. The whole part and the
    /// fraction are both exact in double precision for that range.
    static uint64_t toNanoseconds(double seconds) {
        const double scaled = seconds * kSec2Nano;
        const auto whole = static_cast<uint64_t>(scaled);
        const double fraction = scaled - static_cast<double>(whole);
        return whole + (fraction >= 0.5 ? 1U : 0U);
    }

    uint64_t minTimeNs;
    uint64_t maxTimeNs;
    uint64_t controlPeriodNs;
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
