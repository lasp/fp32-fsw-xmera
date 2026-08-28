#ifndef F32XMERA_DV_ACCUMULATION_ALGORITHM_H
#define F32XMERA_DV_ACCUMULATION_ALGORITHM_H

#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/freestandingIsFinite.hpp"

#include <Eigen/Core>
#include <cstdint>

/*!
 * @brief Validated configuration for the Delta-V accumulator. You cannot change it after
 *        construction. create() makes sure that each parameter is in its limits. If a parameter is
 *        not in its limits, create() throws fsw::invalid_argument.
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

/*! @brief Pure algorithm. It integrates a sample of the body-frame acceleration into an accumulator
 *         of the body-frame Delta-V.
 *
 * Each update() gets one sample of the body-frame acceleration that gravity does not cause. It also
 * gets the accelerometer bias. It subtracts the bias from the sample. Then it integrates the
 * remainder during the configured controlPeriod.
 *
 * The bias is an argument and not a configuration parameter. Thus the caller owns the bias, and the
 * algorithm keeps no calibration state.
 *
 * The algorithm has no time input. The caller must use update() one time in each control period. The
 * caller must also put the time tag on the result.
 *
 * The first update() after construction or after reInitialize() starts the accumulation window. That
 * first update() does not integrate.
 */
class DvAccumulationAlgorithm final {
   public:
    explicit DvAccumulationAlgorithm(const DvAccumulationConfig& config);

    //! Installs the configuration parameters. It does not change the runtime state.
    void setConfig(const DvAccumulationConfig& config);
    //! Sets the accumulator to zero and starts a new accumulation window.
    void reInitialize();
    //! Integrates one sample of the body-frame acceleration. It gives the accumulated Delta-V [m/s].
    //! accelBias_B [m/s^2] is the offset that the measured acceleration contains. update() SUBTRACTS
    //! it from the sample. If you have a correction and not a bias, change the sign of the value
    //! before you give it here. If you do not change the sign, the error doubles and does not cancel.
    //! Use the zero vector for no correction. update() does not validate the bias: a bias that is not
    //! finite gives a Delta-V that is not finite.
    Eigen::Vector3f update(const Eigen::Vector3f& rDDotNoGravity_BN_B, const Eigen::Vector3f& accelBias_B);

   private:
    DvAccumulationConfig cfg;
    Eigen::Vector3f vehAccumDV_B{Eigen::Vector3f::Zero()};  //!< [m/s] Delta-V accumulator
    bool firstCall{true};  //!< [-] the next update() starts the window and does not integrate
};

#endif
