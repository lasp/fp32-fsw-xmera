#ifndef F32XMERA_DV_ACCUMULATION_ALGORITHM_H
#define F32XMERA_DV_ACCUMULATION_ALGORITHM_H

#include "dvAccumulationTypes.h"
#include "msgPayloadDef/AccDataMsgF32Payload.h"

#include <Eigen/Core>
#include <cstdint>

/*! @brief Output of the dvAccumulation algorithm. */
struct DvAccumulationOutput {
    double timeTag{};                                       //!< [s] time-tag of the most-recently-ingested sample
    Eigen::Vector3f vehAccumDV_B{Eigen::Vector3f::Zero()};  //!< [m/s] accumulated Delta-V in body-frame components
};

/*! @brief Pure algorithm: integrates accelerometer packets into a body-frame Delta-V accumulator.
 *
 * On each update() call the input snapshot is sorted by measTime; every packet with measTime
 * strictly greater than the previously-seen latest time is integrated via dt * accel and added
 * to the running accumulator.
 *
 * dvAccumulation has no tunable parameters, so there is no Config. State splits into persistent
 * (previousTime, dvInitialized — carried across a re-initialization so a continuously-running
 * module ignores the backlog already ingested) and non-persistent (vehAccumDV_B).
 */
class DvAccumulationAlgorithm final {
   public:
    DvAccumulationAlgorithm();

    void reInitialize();                        //!< Reset all state: accumulator, previousTime, dvInitialized
    void reInitializeExceptPersistentStates();  //!< Reset only non-persistent state: the accumulator
    DvAccumulationOutput update(const AccDataMsgF32Payload& accData);

   private:
    Eigen::Vector3f vehAccumDV_B{Eigen::Vector3f::Zero()};  //!< [m/s] running Delta-V accumulator (non-persistent)
    uint64_t previousTime{};                                //!< [ns] latest measTime ingested so far (persistent)
    uint32_t dvInitialized{};  //!< [-] non-zero once at least one packet has been ingested (persistent)
};

#endif
