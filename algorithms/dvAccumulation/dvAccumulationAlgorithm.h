#ifndef F32XMERA_DV_ACCUMULATION_ALGORITHM_H
#define F32XMERA_DV_ACCUMULATION_ALGORITHM_H

#include "dvAccumulationTypes.h"
#include "msgPayloadDef/AccDataMsgF32Payload.h"

#include <Eigen/Core>
#include <cstdint>

/*! @brief Validated configuration for the dvAccumulation algorithm.
 *
 * dvAccumulation has no tunable parameters, so the Config class is intentionally empty —
 * its job is to keep the algorithm shape uniform with every other ported algorithm
 * (two-phase init: Config::create() in the adapter's reset(), passed to the algorithm
 * constructor). Construct via DvAccumulationConfig::create(). */
class DvAccumulationConfig final {
   public:
    static DvAccumulationConfig create() { return {}; }

   private:
    DvAccumulationConfig() = default;
};

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
 */
class DvAccumulationAlgorithm final {
   public:
    void resetState(const AccDataMsgF32Payload& accData);
    DvAccumulationOutput update(const AccDataMsgF32Payload& accData);

   private:
    Eigen::Vector3f vehAccumDV_B{Eigen::Vector3f::Zero()};  //!< [m/s] running Delta-V accumulator in body frame
    uint64_t previousTime{};                                //!< [ns] latest measTime ingested so far
    uint32_t dvInitialized{};  //!< [-] non-zero once the accumulator has ingested at least one packet
};

#endif
