#ifndef F32XMERA_DV_ACCUMULATION_H
#define F32XMERA_DV_ACCUMULATION_H

#include "dvAccumulationAlgorithm.h"
#include "msgPayloadDef/IMUSensorBodyMsgF32Payload.h"
#include "msgPayloadDef/NavTransMsgF32Payload.h"

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <memory>

/*! @brief SysModel adapter for DvAccumulationAlgorithm. It reads the body-frame acceleration from
 the input IMU message. Then it uses the algorithm and writes the accumulated body-frame Delta-V to
 the output navigation message. The adapter controls the time. It tags the output message with the
 module call time. */
class DvAccumulation final : public SysModel {
   public:
    void reset(uint64_t callTime) final;
    void updateState(uint64_t callTime) final;
    void reconfigure();   //!< installs the edited properties in the algorithm
    void reInitialize();  //!< sets all the algorithm state to its initial values (state-transition hook)

    // Phase 1: Public config properties — set before reset()
    float controlPeriod = 0.0F;  //!< [s] control period (FSW time step). Set it to more than 0 before reset()
    //! [m/s^2] accelerometer bias that the measured body-frame acceleration contains. The algorithm
    //! subtracts it from each sample. The default value is zero. If the value is zero, the algorithm
    //! makes no correction. The adapter gives the bias to the algorithm on each updateState(). Thus an
    //! edit takes effect on the next call, and reconfigure() is not necessary.
    Eigen::Vector3f accelBias_B = Eigen::Vector3f::Zero();

    Message<NavTransMsgF32Payload> dvAccumulationOutMsg;  //!< accumulated DV output message
    ReadFunctor<IMUSensorBodyMsgF32Payload> imuInMsg;     //!< [-] input IMU body message

   private:
    DvAccumulationConfig toConfig() const;  //!< reset() and reconfigure() both use this function
    std::unique_ptr<DvAccumulationAlgorithm> algorithm = nullptr;
};

#endif
