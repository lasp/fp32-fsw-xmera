#ifndef F32XMERA_DV_MANEUVER_ALGORITHM_H
#define F32XMERA_DV_MANEUVER_ALGORITHM_H

#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/timeConversion.hpp"
#include <stdint.h>
#include <Eigen/Core>
#include <limits>

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
            FSW_THROW_INVALID_ARGUMENT("dvManeuver: minTime must be finite, non-negative, and fit in uint64_t ns.");
        }
        if (!isValidMaxTime(maxTime)) {
            FSW_THROW_INVALID_ARGUMENT("dvManeuver: maxTime must be finite, >= 1 ns, and fit in uint64_t ns.");
        }
        if (!isValidMaxTimeRelativeToMinTime(minTime, maxTime)) {
            FSW_THROW_INVALID_ARGUMENT("dvManeuver: maxTime must be greater than minTime.");
        }
        if (!isValidControlPeriod(controlPeriod)) {
            FSW_THROW_INVALID_ARGUMENT("dvManeuver: controlPeriod must be finite, >= 1 ns, and fit in uint64_t ns.");
        }
        if (!isValidBurnTimeline(maxTime, controlPeriod)) {
            FSW_THROW_INVALID_ARGUMENT("dvManeuver: maxTime plus two controlPeriods must fit in uint64_t ns.");
        }
        if (!isValidCmdForce(cmdForce_B)) {
            FSW_THROW_INVALID_ARGUMENT("dvManeuver: cmdForce_B must be finite.");
        }
        if (!isValidCmdDv(cmdDv_N)) {
            FSW_THROW_INVALID_ARGUMENT("dvManeuver: cmdDv_N must be finite.");
        }
        return {minTime, maxTime, controlPeriod, cmdForce_B, cmdDv_N};
    }

    // Each time must convert to uint64_t ns. The validators reject NaN, Inf, and negative values, and values
    // that overflow uint64_t.
    static bool isValidMinTime(double minTime) { return fsw::secondsToNanos(minTime).has_value(); }
    static bool isValidMaxTime(double maxTime) { return fsw::secondsToNanos(maxTime).value_or(0U) > 0U; }
    /// Compares the rounded nanosecond values, so two limits that round to the same nanosecond are rejected.
    /// Call it only with a valid minTime and maxTime.
    static bool isValidMaxTimeRelativeToMinTime(double minTime, double maxTime) {
        return fsw::secondsToNanos(maxTime).value_or(0U) > fsw::secondsToNanos(minTime).value_or(0U);
    }
    // Each update() adds the period to the burn time. A period that rounds to 0 ns never stops the burn.
    static bool isValidControlPeriod(double controlPeriod) {
        return fsw::secondsToNanos(controlPeriod).value_or(0U) > 0U;
    }
    /// The burn completes at the first multiple of controlPeriod at or after maxTime. This value is a maximum of
    /// maxTime - 1 ns + controlPeriod. On that step, update() adds one more controlPeriod. Thus the burn time does
    /// not wrap the uint64_t ns counter if maxTime - 1 ns + 2 * controlPeriod is in the uint64_t range. Call it only
    /// with a valid maxTime and controlPeriod.
    static bool isValidBurnTimeline(double maxTime, double controlPeriod) {
        const uint64_t maxTimeNs = fsw::secondsToNanos(maxTime).value_or(0U);
        const uint64_t controlPeriodNs = fsw::secondsToNanos(controlPeriod).value_or(0U);
        // Check (maxTimeNs - 1) + 2 * controlPeriodNs <= max without calculating the sum. In uint64_t, a sum that
        // is more than max wraps to a small value, so "sum > max" can never be true. headroomNs is the space that is
        // available above maxTimeNs - 1. For a valid maxTime, maxTimeNs >= 1, thus the two subtractions do not wrap.
        const uint64_t headroomNs = std::numeric_limits<uint64_t>::max() - (maxTimeNs - 1U);
        // 2 * controlPeriodNs can also wrap. Thus compare controlPeriodNs with half of the headroom. For integers,
        // c <= floor(h / 2) is the same test as 2 * c <= h.
        return controlPeriodNs <= headroomNs / 2U;
    }
    static bool isValidCmdForce(const Eigen::Vector3f& cmdForce_B) { return cmdForce_B.allFinite(); }
    static bool isValidCmdDv(const Eigen::Vector3f& cmdDv_N) { return cmdDv_N.allFinite(); }

    double getMinTime() const { return minTime; }
    double getMaxTime() const { return maxTime; }
    double getControlPeriod() const { return controlPeriod; }
    const Eigen::Vector3f& getCmdForce() const { return cmdForce_B; }
    const Eigen::Vector3f& getCmdDv() const { return cmdDv_N; }

   private:
    DvManeuverConfig(double minTime,
                     double maxTime,
                     double controlPeriod,
                     const Eigen::Vector3f& cmdForce_B,
                     const Eigen::Vector3f& cmdDv_N)
        : minTime(minTime), maxTime(maxTime), controlPeriod(controlPeriod), cmdForce_B(cmdForce_B), cmdDv_N(cmdDv_N) {}
    // NOLINTEND(bugprone-easily-swappable-parameters)

    double minTime;
    double maxTime;
    double controlPeriod;
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
    /// @param dvAccumulated_B  Total accumulated delta-V from navigation in body frame components [m/s].
    /// @return Burn state and body force command for this step.
    DvManeuverOutput update(const Eigen::Vector3f& dvAccumulated_B);

   private:
    DvManeuverConfig cfg;
    float cmdDvMagnitude{};      ///< [m/s] magnitude of the configured cmdDv_N, cached by setConfig()
    uint64_t minTimeNs{};        ///< [ns] configured minTime, rounded by setConfig()
    uint64_t maxTimeNs{};        ///< [ns] configured maxTime, rounded by setConfig()
    uint64_t controlPeriodNs{};  ///< [ns] configured controlPeriod, rounded by setConfig()
    DvManeuverBurnState state = DvManeuverBurnState::Executing;  ///< [-] burn state machine state
    uint64_t burnTime{};  ///< [ns] burn time at this update: controlPeriod per executing update so far
};

#endif
