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
 * Each update() call carries one body-frame non-gravitational acceleration sample plus the module
 * call time, integrating dt * accel into the running accumulator over dt = callTime - previousTime.
 *
 * No tunable parameters, so there is no Config — the algorithm is default-constructed.
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
