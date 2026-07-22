#ifndef F32XMERA_DV_EXECUTE_GUIDANCE_ALGORITHM_H
#define F32XMERA_DV_EXECUTE_GUIDANCE_ALGORITHM_H

#include <stdint.h>
#include <Eigen/Core>

/// Burn execution status produced each update. @c commandThrustersOff tells the adapter to write a
/// zeroed thruster on-time command (turning the thrusters off) for this step.
struct DvExecuteGuidanceOutput {
    uint32_t burnExecuting{};    ///< [-] flag indicating whether the burn is in progress
    uint32_t burnComplete{};     ///< [-] flag indicating whether the burn has completed
    bool commandThrustersOff{};  ///< [-] adapter should write a zeroed thruster on-time command this step
};

/// Executes a delta-V burn: compares the accumulated delta-V against the commanded delta-V and,
/// subject to minimum/maximum burn-time gates, decides when the burn is complete and the thrusters
/// must be turned off. The module holds its own burn state machine across updates.
class DvExecuteGuidanceAlgorithm final {
   public:
    /// Resets the burn state machine and normalizes the default control period.
    void reset();

    /// Advances the burn state machine one step.
    /// @param callTime      Evaluation time [ns].
    /// @param vehAccumDV     Total accumulated delta-V from navigation [m/s].
    /// @param dvInrtlCmd     Commanded delta-V in inertial coordinates [m/s].
    /// @param burnStartTime  Commanded burn start time [ns].
    /// @return Burn execution status and thruster-off command flag for this step.
    DvExecuteGuidanceOutput update(uint64_t callTime,
                                   const Eigen::Vector3d& vehAccumDV,
                                   const Eigen::Vector3d& dvInrtlCmd,
                                   uint64_t burnStartTime);

    double minTime{};               ///< [s] minimum burn time allowed to elapse before completion
    double maxTime{};               ///< [s] maximum burn time; 0 disables the maximum-time criterion
    double defaultControlPeriod{};  ///< [s] control period used for the first call

   private:
    Eigen::Vector3d dvInit = Eigen::Vector3d::Zero();  ///< [m/s] accumulated delta-V latched at burn start
    uint32_t burnExecuting{};                          ///< [-] burn currently in progress
    uint32_t burnComplete{};                           ///< [-] burn has completed
    double burnTime{};                                 ///< [s] elapsed burn time
    uint64_t prevCallTime{};                           ///< [ns] previous call time, for the time step
};

#endif
