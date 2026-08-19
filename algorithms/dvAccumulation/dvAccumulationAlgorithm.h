#ifndef F32XMERA_DV_ACCUMULATION_ALGORITHM_H
#define F32XMERA_DV_ACCUMULATION_ALGORITHM_H

#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/freestandingIsFinite.hpp"

#include <Eigen/Core>
#include <cstdint>

/*!
 * @brief Validated, immutable configuration for the Delta-V accumulator. Construct via create(),
 *        which enforces the parameter constraints and throws fsw::invalid_argument on a violation.
 */
class DvAccumulationConfig final {
   public:
    static DvAccumulationConfig create(float controlPeriod) {
        if (!isValidControlPeriod(controlPeriod)) {
            FSW_THROW_INVALID_ARGUMENT("dvAccumulation: controlPeriod must be finite and > 0.");
        }
        return DvAccumulationConfig{controlPeriod};
    }

    static bool isValidControlPeriod(float controlPeriod) {
        return fsw::is_finite(controlPeriod) && controlPeriod > 0.0F;
    }

    float getControlPeriod() const { return this->controlPeriod; }

   private:
    explicit DvAccumulationConfig(float controlPeriod) : controlPeriod(controlPeriod) {}

    float controlPeriod;
};

/*! @brief Pure algorithm: integrates a body-frame acceleration sample into a body-frame Delta-V
 *         accumulator.
 *
 * Each update() call carries one body-frame non-gravitational acceleration sample and integrates it
 * over the configured controlPeriod. The algorithm does not see time: the caller is responsible for
 * driving it once per control period and for time-tagging the result.
 *
 * The first update() after construction or reInitialize() starts the accumulation window rather than
 * integrating: N samples bound N-1 intervals, so the accumulated Delta-V is the acceleration
 * integrated over the elapsed time since that first call.
 */
class DvAccumulationAlgorithm final {
   public:
    explicit DvAccumulationAlgorithm(const DvAccumulationConfig& config);

    //! Install configuration parameters. Does not touch runtime state.
    void setConfig(const DvAccumulationConfig& config);
    //! Re-arm runtime state: zero the accumulator and restart the accumulation window.
    void reInitialize();
    void reInitializeExceptPersistentStates();  //!< Reset only non-persistent state: the accumulator
    //! Integrate one body-frame acceleration sample; returns accumulated Delta-V [m/s].
    Eigen::Vector3f update(const Eigen::Vector3f& rDDotNoGravity_BN_B);

   private:
    DvAccumulationConfig cfg;
    Eigen::Vector3f vehAccumDV_B{Eigen::Vector3f::Zero()};  //!< [m/s] running Delta-V accumulator
    bool firstCall{true};  //!< [-] next update() starts the window instead of integrating
};

#endif
