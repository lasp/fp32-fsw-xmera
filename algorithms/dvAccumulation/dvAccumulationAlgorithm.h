#ifndef F32XMERA_DV_ACCUMULATION_ALGORITHM_H
#define F32XMERA_DV_ACCUMULATION_ALGORITHM_H

#include <Eigen/Core>
#include <cstdint>

/*! @brief Output of the dvAccumulation algorithm. */
struct DvAccumulationOutput {
    double timeTag{};                                       //!< [s] time-tag of the most-recently-ingested sample
    Eigen::Vector3f vehAccumDV_B{Eigen::Vector3f::Zero()};  //!< [m/s] accumulated Delta-V in body-frame components
};

/*! @brief Pure algorithm: integrates a body-frame acceleration sample into a body-frame Delta-V
 *         accumulator.
 *
 * Each update() call carries one body-frame non-gravitational acceleration sample plus the
 * module call time. The time step is dt = callTime - previousTime; the sample is integrated via
 * dt * accel and added to the running accumulator. The first call after a reInitialize()
 * (previousTime == 0) only latches the clock, so dt does not blow up against a zero baseline.
 *
 * dvAccumulation has no tunable parameters, so there is no Config. State splits into persistent
 * (previousTime — carried across a re-initialization so a continuously-running module keeps its
 * time reference) and non-persistent (vehAccumDV_B). previousTime == 0 doubles as the
 * "time reference not yet set" marker for the first update() after a reInitialize().
 */
class DvAccumulationAlgorithm final {
   public:
    DvAccumulationAlgorithm();

    void reInitialize();                        //!< Reset all state: accumulator and previousTime
    void reInitializeExceptPersistentStates();  //!< Reset only non-persistent state: the accumulator
    //! Integrate one body-frame acceleration sample at callTime (ns) into the accumulated Delta-V.
    DvAccumulationOutput update(uint64_t callTime, const Eigen::Vector3f& rDDotNoGravity_BN_B);

   private:
    Eigen::Vector3f vehAccumDV_B{Eigen::Vector3f::Zero()};  //!< [m/s] running Delta-V accumulator (non-persistent)
    uint64_t previousTime{};                                //!< [ns] latest callTime integrated so far (persistent)
};

#endif
