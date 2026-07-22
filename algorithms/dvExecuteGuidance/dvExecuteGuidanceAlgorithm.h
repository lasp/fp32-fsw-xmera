#ifndef F32XMERA_DV_EXECUTE_GUIDANCE_ALGORITHM_H
#define F32XMERA_DV_EXECUTE_GUIDANCE_ALGORITHM_H

#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/freestandingIsFinite.hpp"
#include <stdint.h>
#include <Eigen/Core>

/// Burn execution status produced each update. @c commandThrustersOff tells the adapter to write a
/// zeroed thruster on-time command (turning the thrusters off) for this step.
struct DvExecuteGuidanceOutput {
    uint32_t burnExecuting{};    ///< [-] flag indicating whether the burn is in progress
    uint32_t burnComplete{};     ///< [-] flag indicating whether the burn has completed
    bool commandThrustersOff{};  ///< [-] adapter should write a zeroed thruster on-time command this step
};

/// Validated, immutable configuration for the delta-V burn executor. Construct via create(), which
/// enforces the parameter constraints and throws fsw::invalid_argument on a violation.
class DvExecuteGuidanceConfig final {
   public:
    static DvExecuteGuidanceConfig create(float minTime, float maxTime, float controlPeriod) {
        if (!isValidMinTime(minTime)) {
            FSW_THROW_INVALID_ARGUMENT("dvExecuteGuidance: minTime must be non-negative and finite.");
        }
        if (!isValidMaxTime(maxTime)) {
            FSW_THROW_INVALID_ARGUMENT("dvExecuteGuidance: maxTime must be non-negative and finite.");
        }
        if (!isValidControlPeriod(controlPeriod)) {
            FSW_THROW_INVALID_ARGUMENT("dvExecuteGuidance: controlPeriod must be positive and finite.");
        }
        return {minTime, maxTime, controlPeriod};
    }

    static bool isValidMinTime(float minTime) { return minTime >= 0.0F && fsw::is_finite(minTime); }
    static bool isValidMaxTime(float maxTime) { return maxTime >= 0.0F && fsw::is_finite(maxTime); }
    static bool isValidControlPeriod(float controlPeriod) {
        return controlPeriod > 0.0F && fsw::is_finite(controlPeriod);
    }

    float getMinTime() const { return minTime; }
    float getMaxTime() const { return maxTime; }
    float getControlPeriod() const { return controlPeriod; }

   private:
    DvExecuteGuidanceConfig(float minTime, float maxTime, float controlPeriod)
        : minTime(minTime), maxTime(maxTime), controlPeriod(controlPeriod) {}

    float minTime;
    float maxTime;
    float controlPeriod;
};

/// Executes a delta-V burn: compares the accumulated delta-V against the commanded delta-V and,
/// subject to minimum/maximum burn-time gates, decides when the burn is complete and the thrusters
/// must be turned off. The module holds its own burn state machine across updates.
class DvExecuteGuidanceAlgorithm final {
   public:
    explicit DvExecuteGuidanceAlgorithm(const DvExecuteGuidanceConfig& config);

    /// Installs the configuration parameters. Does not touch runtime state.
    void setConfig(const DvExecuteGuidanceConfig& config);

    /// Resets the burn state machine to its initial (pre-burn) condition.
    void reInitialize();

    /// Advances the burn state machine one step.
    /// @param callTime      Evaluation time [ns].
    /// @param vehAccumDV     Total accumulated delta-V from navigation [m/s].
    /// @param dvInrtlCmd     Commanded delta-V in inertial coordinates [m/s].
    /// @param burnStartTime  Commanded burn start time [ns].
    /// @return Burn execution status and thruster-off command flag for this step.
    DvExecuteGuidanceOutput update(uint64_t callTime,
                                   const Eigen::Vector3f& vehAccumDV,
                                   const Eigen::Vector3f& dvInrtlCmd,
                                   uint64_t burnStartTime);

   private:
    DvExecuteGuidanceConfig cfg;
    Eigen::Vector3f dvInit = Eigen::Vector3f::Zero();  ///< [m/s] accumulated delta-V latched at burn start
    uint32_t burnExecuting{};                          ///< [-] burn currently in progress
    uint32_t burnComplete{};                           ///< [-] burn has completed
    float burnTime{};                                  ///< [s] elapsed burn time
};

#endif
