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
    static DvAccumulationConfig create(float controlPeriod, const Eigen::Vector3f& accelBias_B) {
        if (!isValidControlPeriod(controlPeriod)) {
            FSW_THROW_INVALID_ARGUMENT("dvAccumulation: controlPeriod must be finite and > 0.");
        }
        if (!isValidAccelBias(accelBias_B)) {
            FSW_THROW_INVALID_ARGUMENT("dvAccumulation: accelBias_B must be finite.");
        }
        return DvAccumulationConfig{controlPeriod, accelBias_B};
    }

    static bool isValidControlPeriod(float controlPeriod) {
        return fsw::is_finite(controlPeriod) && controlPeriod > 0.0F;
    }

    static bool isValidAccelBias(const Eigen::Vector3f& accelBias_B) { return accelBias_B.allFinite(); }

    float getControlPeriod() const { return this->controlPeriod; }
    const Eigen::Vector3f& getAccelBias_B() const { return this->accelBias_B; }

   private:
    DvAccumulationConfig(float controlPeriod, const Eigen::Vector3f& accelBias_B)
        : controlPeriod(controlPeriod), accelBias_B(accelBias_B) {}

    float controlPeriod;
    //! [m/s^2] offset that the measured acceleration contains. update() SUBTRACTS it from each sample.
    Eigen::Vector3f accelBias_B;
};

/*! @brief Pure algorithm. It integrates a sample of the body-frame acceleration into an accumulator
 *         of the body-frame Delta-V.
 *
 * Each update() gets one sample of the body-frame acceleration that gravity does not cause. It
 * subtracts the configured accelerometer bias from the sample. Then it integrates the remainder during
 * the configured controlPeriod.
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
    //! Integrates one sample of the body-frame acceleration less the configured bias. It gives the
    //! accumulated Delta-V [m/s].
    Eigen::Vector3f update(const Eigen::Vector3f& rDDotNoGravity_BN_B);

   private:
    DvAccumulationConfig cfg;
    Eigen::Vector3f vehAccumDV_B{Eigen::Vector3f::Zero()};  //!< [m/s] Delta-V accumulator
    bool firstCall{true};  //!< [-] the next update() starts the window and does not integrate
};

#endif
