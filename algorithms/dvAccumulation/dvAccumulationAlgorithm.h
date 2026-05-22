#ifndef F32XMERA_DV_ACCUMULATION_ALGORITHM_H
#define F32XMERA_DV_ACCUMULATION_ALGORITHM_H

#include "dvAccumulationTypes.h"
#include "msgPayloadDef/AccDataMsgF32Payload.h"

#include <cstdint>

/*! @brief Output of the dvAccumulation algorithm. */
struct DvAccumulationOutput {
    double timeTag{};          //!< [s] time-tag of the most-recently-ingested sample
    double vehAccumDV_B[3]{};  //!< [m/s] accumulated Delta-V in body-frame components
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
    double vehAccumDV_B[3]{};  //!< [m/s] running Delta-V accumulator in body frame
    uint64_t previousTime{};   //!< [ns] latest measTime ingested so far
    uint32_t dvInitialized{};  //!< [-] non-zero once the accumulator has ingested at least one packet
};

#endif
